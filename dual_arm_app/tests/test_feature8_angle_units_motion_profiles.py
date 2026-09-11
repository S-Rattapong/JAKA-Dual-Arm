"""Feature 8 angle presentation and relative Center motion-profile contracts.

All coverage is pure/offline. No Web motion route, ROS service, driver, or robot is used.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import tempfile

import pytest

from dual_arm_app.backend.center_path_store import (
    CenterPathStore,
    CenterPathStoreError,
    normalize_center_path_payload,
)
from dual_arm_app.backend.object_global_planning import (
    ObjectGlobalPlanInputError,
    RELATIVE_WAYPOINT_PROFILE_TIMING_SEMANTIC,
    normalize_object_global_plan_request,
    plan_object_global,
    relative_segment_duration_s,
)
from dual_arm_app.backend.object_grasp_model import SYNTHETIC_OBJECT_GRASP_FIXTURE
from dual_arm_app.backend.object_trajectory_ik import (
    ArmIkSolution,
    CanonicalJointPositionLimits,
    CombinedStateValidity,
)


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
ANGLE_UNITS = WEB / "digital_twin_angle_units.js"
PLANNING_JS = WEB / "digital_twin_phase3_planning.js"
CONTROLLER = WEB / "digital_twin.js"
HTML = WEB / "index.html"
PLANNER = ROOT / "dual_arm_app/backend/object_global_planning.py"


def run_node(source: str) -> dict:
    result = subprocess.run(
        ["node", "--input-type=module", "-e", source],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(result.stdout)


def center_path_payload(*, profiles: bool = True) -> dict:
    waypoints = [
        {"identifier": "old-a", "translation_m": [0.0, 0.0, 0.8], "rpy_rad": [0.7, 0.0, 0.0]},
        {"identifier": "old-b", "translation_m": [0.2, 0.0, 0.9], "rpy_rad": [0.0, 0.1, 0.0]},
        {"identifier": "old-c", "translation_m": [0.4, 0.0, 1.0], "rpy_rad": [0.0, 0.0, -0.2]},
    ]
    if profiles:
        waypoints[0].update(speed_percent=50.0, acceleration_percent=100.0)
        waypoints[1].update(speed_percent=100.0, acceleration_percent=25.0)
        waypoints[2].update(speed_percent=100.0, acceleration_percent=100.0)
    return {
        "waypoints": waypoints,
        "fixed_orientation_rpy_rad": [0.0, 0.0, 0.0],
        "orientation_source": "FEATURE 8 TEST",
        "segment_duration_s": 1.0,
        "samples_per_segment": 2,
        "candidate_attempts_per_arm": 3,
    }


def planning_payload(*, profiles: bool = True) -> dict:
    payload = center_path_payload(profiles=profiles)
    payload.update({
        "name": "Feature8_Offline",
        "expected_grasp_content_revision": "sha256:test-grasp",
        "expected_lock_generation": 1,
        "expected_lock_revision": "sha256:test-lock",
        "expected_calibration_revision": "sha256:test-calibration",
        "expected_model_calibration_revision": "sha256:test-calibration",
    })
    return payload


PLANNING_AUTHORITY = {
    "grasp_status": "GRASP_LOCKED",
    "grasp_source": "LOCKED OPERATOR GRASP",
    "grasp_content_revision": "sha256:test-grasp",
    "lock_generation": 1,
    "lock_revision": "sha256:test-lock",
    "left": {"translation_m": [0.0, 0.25, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
    "right": {"translation_m": [0.0, -0.25, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
    "calibration_revision": "sha256:test-calibration",
    "model_calibration_revision": "sha256:test-calibration",
    "calibration_revision_status": "MATCH",
    "calibration_state": "MODEL_DEFAULT",
    "physical_calibration": "NOT_CALIBRATED",
    "physically_calibrated": False,
    "tcp_tool_contract_status": "UNVERIFIED",
}


class EchoPlanningAdapter:
    def solve_arm_ik(self, **_kwargs):
        return ArmIkSolution(True, (0.0,) * 6, "feature8 offline synthetic success")

    def check_combined_state(self, **_kwargs):
        return CombinedStateValidity(True, "feature8 offline synthetic valid")


def run_plan(payload: dict) -> dict:
    return plan_object_global(
        payload,
        EchoPlanningAdapter(),
        CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12),
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        planning_authority=PLANNING_AUTHORITY,
    )


class TestFeature8AngleUnits:
    def test_exact_conversion_repeated_switches_keep_canonical_radians(self):
        result = run_node(r'''
import {
  ANGLE_UNIT_DEGREES, ANGLE_UNIT_RADIANS,
  presentCanonicalRadians, radiansToDegrees, rotationDisplayToRadians,
} from "./dual_arm_app/web/digital_twin_angle_units.js";
const canonical = [0.7, -0.2, Math.PI];
const original = JSON.stringify(canonical);
const immediateDegrees = presentCanonicalRadians(canonical, ANGLE_UNIT_DEGREES);
let displayed = null;
for (let index = 0; index < 100; index += 1) {
  displayed = presentCanonicalRadians(
    canonical,
    index % 2 === 0 ? ANGLE_UNIT_RADIANS : ANGLE_UNIT_DEGREES,
  );
}
console.log(JSON.stringify({
  degrees: radiansToDegrees(0.7),
  roundTrip: rotationDisplayToRadians(immediateDegrees[0], ANGLE_UNIT_DEGREES),
  unchanged: original === JSON.stringify(canonical),
  displayed,
}));
''')
        assert result["degrees"] == pytest.approx(40.10704565915762)
        assert result["roundTrip"] == pytest.approx(0.7)
        assert result["unchanged"] is True
        assert result["displayed"][0] == pytest.approx(40.10704565915762)

    def test_global_selector_choices_and_presentation_only_controller(self):
        html = HTML.read_text(encoding="utf-8")
        controller = CONTROLLER.read_text(encoding="utf-8")
        assert html.count('id="digitalTwinRotationUnit"') == 1
        assert ">Degrees (°)<" in html
        assert ">Radians (rad)<" in html
        body = controller.split("function setGlobalAngleUnit", 1)[1].split(
            "function bindGlobalAngleUnitControl", 1
        )[0]
        for immediate_rerender in (
            "writeRigidGraspEditorFields", "writeObjectPoseControls",
            "writeNewObjectWaypointRotationControls", "updateObjectGraspRelativeUi",
            "updateApproachWaypointUi", "renderObjectWaypointRows",
        ):
            assert immediate_rerender in body
        for authority_mutation in (
            "invalidatePlanningForGraspChange", "invalidateAllPhase4Validation",
            "clearPlannedTrajectory", "requestGeneration += 1",
        ):
            assert authority_mutation not in body
        assert "Presentation-only by design" in body

    def test_operator_fields_use_global_helpers_while_canonical_names_stay_radians(self):
        controller = CONTROLLER.read_text(encoding="utf-8")
        for token in (
            "rotationRadiansToDisplay", "rotationDisplayToRadians",
            "formatRotationRadians", "rpyRad", "rpy_rad",
        ):
            assert token in controller
        assert "digital_twin_angle_units.js" in controller
        assert "canonical planning remains radians" in HTML.read_text(encoding="utf-8")


class TestFeature8WaypointProfiles:
    def test_default_duplicate_reorder_and_edit_preserve_profiles(self):
        result = run_node(r'''
import {
  duplicateObjectPlanningWaypoint, normalizeObjectPlanningWaypoints,
  reorderObjectPlanningWaypoint,
} from "./dual_arm_app/web/digital_twin_phase3_planning.js";
const pose = (id, x, speed, acceleration) => ({
  identifier:id, translation_m:[x,0,0], rpy_rad:[0,0,0],
  ...(speed === undefined ? {} : {speed_percent:speed}),
  ...(acceleration === undefined ? {} : {acceleration_percent:acceleration}),
});
const defaults = normalizeObjectPlanningWaypoints([pose("legacy",0)]);
const base = normalizeObjectPlanningWaypoints([
  pose("a",0,40,81), pose("b",1,60,64), pose("c",2,80,49),
]);
const duplicated = duplicateObjectPlanningWaypoint(base, 0);
const reordered = reorderObjectPlanningWaypoint(duplicated, 3, 1);
const edited = normalizeObjectPlanningWaypoints(base.map((item, index) => (
  index === 1 ? {...item, speed_percent:55, acceleration_percent:36} : item
)));
console.log(JSON.stringify({defaults, duplicated, reordered, edited}));
''')
        assert result["defaults"][0]["speed_percent"] == 100
        assert result["defaults"][0]["acceleration_percent"] == 100
        assert result["duplicated"][1]["speed_percent"] == 40
        assert result["duplicated"][1]["acceleration_percent"] == 81
        assert [item["speed_percent"] for item in result["reordered"]] == [40, 80, 40, 60]
        assert result["edited"][1]["speed_percent"] == 55
        assert result["edited"][1]["acceleration_percent"] == 36

    def test_slider_numeric_sync_final_state_and_invalid_rejection_contracts(self):
        controller = CONTROLLER.read_text(encoding="utf-8")
        html = HTML.read_text(encoding="utf-8")
        rows = controller.split("function renderObjectWaypointRows", 1)[1].split(
            "function objectWaypointFromAddControls", 1
        )[0]
        for token in (
            'range.type = "range"', 'numeric.type = "number"',
            'range.addEventListener("input"', "numeric.value = range.value",
            'numeric.addEventListener("input"', "range.value = numeric.value",
            "range.disabled = true", "numeric.disabled = true",
            'note.textContent = "Final — no outgoing segment"',
        ):
            assert token in rows
        assert "Wn profile applies to Wn → Wn+1" in html
        assert "relative planning scales, not manufacturer" in html
        invalid = run_node(r'''
import {normalizeObjectPlanningWaypoints} from "./dual_arm_app/web/digital_twin_phase3_planning.js";
const make = (speed, acceleration) => [{
  identifier:"W0", translation_m:[0,0,0], rpy_rad:[0,0,0],
  speed_percent:speed, acceleration_percent:acceleration,
}];
const errors = [];
for (const values of [[0,100],[101,100],[100,0],[100,-1]]) {
  try { normalizeObjectPlanningWaypoints(make(...values)); }
  catch (error) { errors.push(error.message); }
}
console.log(JSON.stringify({errors}));
''')
        assert len(invalid["errors"]) == 4
        assert all("(0, 100]" in message for message in invalid["errors"])

    def test_save_load_persistence_and_legacy_defaults(self):
        legacy = normalize_center_path_payload(center_path_payload(profiles=False))
        assert all(item["speed_percent"] == 100.0 for item in legacy["waypoints"])
        assert all(item["acceleration_percent"] == 100.0 for item in legacy["waypoints"])

        store_dir = Path(tempfile.mkdtemp(prefix="feature8_center_path_"))
        store = CenterPathStore(store_dir)
        store.save("feature8_profiles", center_path_payload(), overwrite=True)
        loaded = store.load("feature8_profiles")["document"]["path"]
        assert [item["speed_percent"] for item in loaded["waypoints"]] == [50.0, 100.0, 100.0]
        assert [item["acceleration_percent"] for item in loaded["waypoints"]] == [100.0, 25.0, 100.0]

        for field, value in (("speed_percent", 0), ("acceleration_percent", 101)):
            invalid = center_path_payload()
            invalid["waypoints"][0][field] = value
            with pytest.raises(CenterPathStoreError, match=r"within \(0, 100\]"):
                normalize_center_path_payload(invalid)


class TestFeature8PlannerTiming:
    @pytest.mark.parametrize(
        ("speed", "acceleration", "factor"),
        ((100, 100, 1.0), (50, 100, 2.0), (100, 25, 2.0), (80, 25, 2.0)),
    )
    def test_conservative_relative_timing_formula(self, speed, acceleration, factor):
        assert relative_segment_duration_s(1.25, speed, acceleration) == 1.25 * factor
        assert relative_segment_duration_s(1.25, speed, acceleration) >= 1.25

    def test_invalid_profiles_fail_instead_of_clamp(self):
        for speed, acceleration in ((0, 100), (-1, 100), (101, 100), (100, 0), (100, 101)):
            with pytest.raises(ObjectGlobalPlanInputError, match=r"within \(0, 100\]"):
                relative_segment_duration_s(1.0, speed, acceleration)

    def test_option_b_association_and_final_waypoint_has_no_outgoing_segment(self):
        normalized = normalize_object_global_plan_request(planning_payload())
        assert normalized["segment_duration_s"] == 1.0
        assert normalized["segment_durations_s"] == (2.0, 2.0)
        assert normalized["waypoint_profiles"][0]["speed_percent"] == 50.0
        assert normalized["waypoint_profiles"][1]["acceleration_percent"] == 25.0
        assert len(normalized["segment_durations_s"]) == len(normalized["waypoints"]) - 1

    def test_global_planner_timestamps_metadata_and_spatial_architecture(self):
        baseline = run_plan(planning_payload(profiles=False))
        stretched = run_plan(planning_payload(profiles=True))
        assert baseline["requested_segment_durations_s"] == [1.0, 1.0]
        assert stretched["requested_segment_durations_s"] == [2.0, 2.0]
        assert baseline["segment_durations_s"] == [1.368, 1.368]
        assert stretched["segment_durations_s"] == [2.376, 2.376]
        assert baseline["velocity_shaping"]["waypoint_times_s"] == [0.0, 1.368, 2.736]
        assert stretched["velocity_shaping"]["waypoint_times_s"] == [0.0, 2.376, 4.752]
        assert stretched["common_timestamps_s"] == [
            point["time_from_start_s"] for point in stretched["global_path"]
        ]
        assert [item["profile_source_waypoint"] for item in stretched["segment_timing"]] == ["old-a", "old-b"]
        assert [item["from_waypoint"] for item in stretched["segment_timing"]] == ["old-a", "old-b"]
        assert [item["to_waypoint"] for item in stretched["segment_timing"]] == ["old-b", "old-c"]
        assert stretched["waypoints"][-1]["has_outgoing_segment"] is False
        assert stretched["timing_semantic"] == RELATIVE_WAYPOINT_PROFILE_TIMING_SEMANTIC
        baseline_waypoints = [
            baseline["object_samples"][tick]["object_pose"]
            for tick in baseline["velocity_shaping"]["waypoint_ticks"]
        ]
        stretched_waypoints = [
            stretched["object_samples"][tick]["object_pose"]
            for tick in stretched["velocity_shaping"]["waypoint_ticks"]
        ]
        assert baseline_waypoints == stretched_waypoints
        assert baseline["graph"]["layer_count"] == stretched["graph"]["layer_count"]

    def test_no_derivatives_or_new_timing_dependency_and_approach_baseline_unchanged(self):
        planner = PLANNER.read_text(encoding="utf-8")
        timing_call = planner.split("generate_multi_waypoint_object_trajectory(", 1)[1].split(")", 1)[0]
        assert 'segment_durations_s=request["segment_durations_s"]' in timing_call
        approach = planner.split("def _build_independent_approach_trajectory", 1)[1].split(
            "def _approach_selected_path_payload", 1
        )[0]
        assert 'duration = request["segment_duration_s"]' in approach
        for forbidden in ("Ruckig", "TOPP-RA", "joint_velocity", "joint_acceleration"):
            assert forbidden not in planner
        assert "generate_multi_waypoint_object_trajectory" in planner
        assert "build_object_trajectory_candidate_graph" in planner
        assert "compare_greedy_and_global" in planner
