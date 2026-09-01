"""Offline Web contract tests for PRE-P5.B rigid-grasp customization."""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from html.parser import HTMLParser
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
HTML = WEB / "index.html"
CONTROLLER = WEB / "digital_twin.js"
PREVIEW = WEB / "digital_twin_object_grasp_preview.js"
SOURCE = WEB / "digital_twin_grasp_configuration_source.js"


class _Ids(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []

    def handle_starttag(self, _tag, attrs):
        self.ids.extend(value for key, value in attrs if key == "id")


class CustomRigidGraspWebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML.read_text(encoding="utf-8")
        cls.controller = CONTROLLER.read_text(encoding="utf-8")
        cls.preview = PREVIEW.read_text(encoding="utf-8")
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_left_and_right_xyz_rpy_degree_fields_and_lock_controls_exist(self):
        for side in ("Left", "Right"):
            for suffix in ("X", "Y", "Z", "RollDeg", "PitchDeg", "YawDeg"):
                self.assertIn(f'id="digitalTwin{side}Grasp{suffix}"', self.html)
        for identifier in (
            "digitalTwinLockGraspConfiguration",
            "digitalTwinUnlockGraspConfiguration",
            "digitalTwinGraspConfigurationState",
            "digitalTwinGraspRevision",
            "digitalTwinGraspLockGeneration",
        ):
            self.assertIn(f'id="{identifier}"', self.html)
        self.assertIn("Roll [deg]", self.html)
        self.assertIn("Pitch [deg]", self.html)
        self.assertIn("Yaw [deg]", self.html)

    def test_selected_frame_translate_rotate_and_scale_disabled_semantics_exist(self):
        for value in ("CENTER", "LEFT_GRASP", "RIGHT_GRASP"):
            self.assertIn(f'value="{value}"', self.html)
        self.assertIn('id="digitalTwinGraspGizmoTranslate"', self.html)
        self.assertIn('id="digitalTwinGraspGizmoRotate"', self.html)
        self.assertIn("SCALE DISABLED", self.html)
        self.assertIn('objectTransformControls.setMode(rigidGraspState.gizmoMode)', self.controller)
        self.assertIn('? "world" : "local"', self.controller)

    def test_locked_and_unlocked_ui_is_fail_closed_but_center_remains_available(self):
        update = self.controller.split(
            "function updateRigidGraspConfigurationUi()", 1
        )[1].split("function applyBackendRigidGraspState", 1)[0]
        self.assertIn('rigidGraspState.state === "GRASP_UNLOCKED"', update)
        self.assertIn("element.disabled = !editable", update)
        self.assertIn('rigidGraspState.state !== "GRASP_LOCKED"', update)
        select = self.controller.split("function setRigidGraspSelectedFrame", 1)[1].split(
            "function setRigidGraspGizmoMode", 1
        )[0]
        self.assertIn("GRASP_SELECTED_FRAMES.CENTER", select)
        self.assertIn("disabled while locked", select)

    def test_save_waypoint_and_phase3_are_blocked_until_required_state(self):
        waypoint_ui = self.controller.split(
            "function updateObjectWaypointPlanningUi()", 1
        )[1].split("function setObjectPlanningWaypoints", 1)[0]
        self.assertIn('button.disabled = !locked', waypoint_ui)
        self.assertIn("planButton.disabled = state.planning || !prerequisitesReady", waypoint_ui)
        planner = self.controller.split("async function planObjectGlobal", 1)[1].split(
            "function bindObjectWaypointPlanningControls", 1
        )[0]
        self.assertIn("phase3PlanningAuthorityIdentity()", planner)
        self.assertIn("expectedGraspContentRevision", planner)
        self.assertIn('planButton.textContent = state.planning', self.controller)
        self.assertIn('"Generate Trajectory"', self.controller)

    def test_grasp_change_invalidates_plan_ghost_phase4_but_not_waypoints(self):
        invalidation = self.controller.split(
            "function invalidatePlanningForGraspChange", 1
        )[1].split("function updateRigidGraspDraftSide", 1)[0]
        self.assertIn('objectGlobalPlanState.status = "INVALIDATED"', invalidation)
        self.assertIn("clearPlannedTrajectory()", invalidation)
        self.assertIn("invalidateAllPhase4Validation(reason)", invalidation)
        self.assertNotIn("objectWaypointPlanningState.waypoints", invalidation)
        self.assertIn("RIGID_GRASP_NOT_LOCKED", self.controller)
        self.assertIn("PLAN_GRASP_REVISION_STALE", self.controller)

    def test_left_and_right_updates_are_local_and_side_isolated(self):
        update = self.controller.split("function updateRigidGraspDraftSide", 1)[1].split(
            "function graspPoseFromNumericControls", 1
        )[0]
        self.assertIn("[side]: copyGraspPose(normalized)", update)
        self.assertIn("LOCAL_DRAFT_MODIFIED — LOCK REQUIRED", update)
        gizmo = self.controller.split('objectTransformControls.addEventListener("objectChange"', 1)[1].split(
            'objectTransformControls.addEventListener("change"', 1
        )[0]
        self.assertIn("inverseRigidMatrix4(worldTObject)", gizmo)
        self.assertIn("updateRigidGraspDraftSide", gizmo)

    def test_nonidentity_center_composition_and_rpy_convention(self):
        with tempfile.TemporaryDirectory() as directory:
            module = Path(directory) / "preview.mjs"
            module.write_text(self.preview, encoding="utf-8")
            script = f"""
              import {{
                computeObjectGraspRelativeState,
                computeWorldGraspFrameMatrices,
                matrix4FromTranslationRpy,
                multiplyMatrix4,
              }} from {json.dumps(module.as_uri())};
              const draft = {{
                left: {{translationM:[0.1,0.2,0.3], rpyRad:[0.2,-0.3,0.4]}},
                right: {{translationM:[-0.1,-0.25,0.05], rpyRad:[-0.4,0.1,-0.2]}},
              }};
              const a = {{translationM:[1.0,-2.0,0.7], rpyRad:[0.3,-0.2,0.8]}};
              const b = {{translationM:[-0.4,0.6,2.1], rpyRad:[-0.5,0.4,-0.7]}};
              const before = JSON.stringify(computeObjectGraspRelativeState(draft));
              const worldA = computeWorldGraspFrameMatrices(a, draft);
              const worldB = computeWorldGraspFrameMatrices(b, draft);
              const expectedLeft = multiplyMatrix4(
                matrix4FromTranslationRpy(a), matrix4FromTranslationRpy(draft.left),
              );
              const after = JSON.stringify(computeObjectGraspRelativeState(draft));
              const yaw = matrix4FromTranslationRpy({{translationM:[0,0,0],rpyRad:[0,0,Math.PI/2]}});
              console.log(JSON.stringify({{
                localInvariant: before === after,
                leftMatches: JSON.stringify(worldA.worldTLeft) === JSON.stringify(expectedLeft),
                worldsDiffer: JSON.stringify(worldA.worldTLeft) !== JSON.stringify(worldB.worldTLeft),
                yawX: [yaw[0][0], yaw[1][0], yaw[2][0]],
              }}));
            """
            result = subprocess.run(
                ["node", "--input-type=module", "--eval", script],
                check=True,
                capture_output=True,
                text=True,
            )
        output = json.loads(result.stdout)
        self.assertTrue(output["localInvariant"])
        self.assertTrue(output["leftMatches"])
        self.assertTrue(output["worldsDiffer"])
        self.assertAlmostEqual(output["yawX"][0], 0.0, places=12)
        self.assertAlmostEqual(output["yawX"][1], 1.0, places=12)

    def test_backend_is_runtime_authority_and_transport_has_only_state_routes(self):
        self.assertIn("requestGraspConfiguration", self.controller)
        self.assertNotIn("SYNTHETIC_OBJECT_T_LEFT_POSE", self.controller)
        self.assertNotIn("SYNTHETIC_OBJECT_T_RIGHT_POSE", self.controller)
        self.assertIn("SYNTHETIC_DEFAULT_DRAFT", self.preview)
        for endpoint in (
            "/api/digital-twin/grasp-configuration",
            "/api/digital-twin/grasp-configuration/lock",
            "/api/digital-twin/grasp-configuration/unlock",
        ):
            self.assertIn(endpoint, self.source)
        for forbidden in (
            "/api/program/run", "/api/jog", "/api/home", "joint_move",
            "linear_move", "servo",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_html_ids_remain_unique(self):
        parser = _Ids()
        parser.feed(self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))


if __name__ == "__main__":
    unittest.main()
