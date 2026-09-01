"""Focused offline tests for authoritative custom-grasp planning and validation."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import patch

from dual_arm_app.backend.object_global_planning import plan_object_global
from dual_arm_app.backend.object_grasp_model import ObjectGraspModel, RigidTransform
from dual_arm_app.backend.object_trajectory_ik import (
    ArmIkSolution,
    CanonicalJointPositionLimits,
    CombinedStateValidity,
)
from dual_arm_app.backend.phase4_trajectory_validation import (
    load_moveit_joint_dynamics_limits,
    validate_phase4_trajectory,
)
from dual_arm_app.backend.phase4_unified_validation import compute_plan_fingerprint
from dual_arm_app.backend.rigid_grasp_configuration import AuthoritativeRigidGraspState
from dual_arm_app.backend.world_frame_calibration import get_world_calibration_revision
from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import


CALIBRATION_REVISION = get_world_calibration_revision()
LIMITS = CanonicalJointPositionLimits((-6.28,) * 12, (6.28,) * 12)
ROOT = Path(__file__).resolve().parents[2]


class RecordingPlanningAdapter:
    def __init__(self):
        self.ik_calls = 0
        self.targets = []

    def unavailable_services(self):
        return []

    def solve_arm_ik(self, **kwargs):
        self.ik_calls += 1
        self.targets.append((kwargs["group_name"], kwargs["target_world_T_tip"]))
        return ArmIkSolution(True, (0.0,) * 6, "offline deterministic")

    def check_combined_state(self, *, joint_positions_rad, group_name):
        del joint_positions_rad, group_name
        return CombinedStateValidity(True, "offline valid")


class PlannedTargetFk:
    def __init__(self, plan):
        self.targets = [
            (
                RigidTransform.from_matrix(sample["left_target"]["matrix"]),
                RigidTransform.from_matrix(sample["right_target"]["matrix"]),
            )
            for sample in plan["object_samples"]
        ]
        self.index = 0

    def compute_combined_fk(self, joint_positions_rad, timeout_s=2.0):
        del joint_positions_rad, timeout_s
        left, right = self.targets[self.index]
        self.index += 1
        return {"status": "PASS", "left": left, "right": right}


def locked_payload():
    return {
        "left": {
            "translation_m": [0.12, 0.31, -0.04],
            "rpy_rad": [0.2, -0.1, 0.3],
        },
        "right": {
            "translation_m": [-0.08, -0.29, 0.06],
            "rpy_rad": [-0.15, 0.25, -0.2],
        },
    }


def request_for(snapshot):
    return {
        "name": "PRE_P5_C_TEST",
        "waypoints": [
            {"identifier": "W0", "translation_m": [0.0, 0.0, 0.8]},
            {"identifier": "W1", "translation_m": [0.2, -0.1, 0.9]},
        ],
        "fixed_orientation_rpy_rad": [0.1, -0.2, 0.3],
        "segment_duration_s": 1.0,
        "samples_per_segment": 3,
        "candidate_attempts_per_arm": 1,
        "expected_grasp_content_revision": snapshot["content_revision"],
        "expected_lock_generation": snapshot["lock_generation"],
        "expected_lock_revision": snapshot["lock_revision"],
        "expected_calibration_revision": CALIBRATION_REVISION,
        "expected_model_calibration_revision": CALIBRATION_REVISION,
    }


def authority_for(snapshot, payload):
    return {
        "grasp_status": "GRASP_LOCKED",
        "grasp_source": "LOCKED OPERATOR GRASP",
        "grasp_content_revision": snapshot["content_revision"],
        "lock_generation": snapshot["lock_generation"],
        "lock_revision": snapshot["lock_revision"],
        "left": payload["left"],
        "right": payload["right"],
        "calibration_revision": CALIBRATION_REVISION,
        "model_calibration_revision": CALIBRATION_REVISION,
        "calibration_revision_status": "MATCH",
        "calibration_state": "MODEL_DEFAULT",
        "physical_calibration": "NOT_CALIBRATED",
        "physically_calibrated": False,
        "tcp_tool_contract_status": "UNVERIFIED",
    }


def pure_custom_plan(payload=None):
    payload = payload or locked_payload()
    state = AuthoritativeRigidGraspState()
    locked = state.lock(payload)["locked_snapshot"]
    adapter = RecordingPlanningAdapter()
    plan = plan_object_global(
        request_for(locked),
        adapter,
        LIMITS,
        grasp_model=ObjectGraspModel(
            RigidTransform.from_translation_rpy(
                payload["left"]["translation_m"], payload["left"]["rpy_rad"]
            ),
            RigidTransform.from_translation_rpy(
                payload["right"]["translation_m"], payload["right"]["rpy_rad"]
            ),
        ),
        planning_authority=authority_for(locked, payload),
    )
    return plan, adapter


class AuthoritativePhase3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def setUp(self):
        self.node = self.backend.node
        self.node.rigid_grasp_configuration = AuthoritativeRigidGraspState()
        self.node.object_global_planning_joint_limits = LIMITS
        self.node.object_global_planning_adapter = RecordingPlanningAdapter()
        self.node.object_global_planning_error = None

    def lock(self):
        return self.node.lock_digital_twin_grasp_configuration(
            locked_payload()
        )["locked_snapshot"]

    def test_unlocked_and_missing_locked_snapshot_reject_before_ik(self):
        draft = self.node.rigid_grasp_configuration.snapshot()
        request = {
            **request_for({
                "content_revision": draft["draft_content_revision"],
                "lock_generation": 1,
                "lock_revision": "sha256:missing",
            })
        }
        result = self.node.plan_digital_twin_object_global(request)
        self.assertEqual(result["planner_status"], "REJECTED")
        self.assertEqual(result["failure"]["reason"], "RIGID_GRASP_UNLOCKED_OR_MISSING")
        self.assertEqual(self.node.object_global_planning_adapter.ik_calls, 0)

    def test_every_expected_revision_mismatch_rejects_before_ik(self):
        locked = self.lock()
        mutations = {
            "expected_grasp_content_revision": "sha256:old-grasp",
            "expected_lock_generation": locked["lock_generation"] + 1,
            "expected_lock_revision": "sha256:old-lock",
            "expected_calibration_revision": "sha256:old-calibration",
            "expected_model_calibration_revision": "sha256:old-model",
        }
        for field, value in mutations.items():
            self.node.object_global_planning_adapter = RecordingPlanningAdapter()
            request = request_for(locked)
            request[field] = value
            with self.subTest(field=field):
                result = self.node.plan_digital_twin_object_global(request)
                self.assertEqual(result["planner_status"], "REJECTED")
                self.assertEqual(self.node.object_global_planning_adapter.ik_calls, 0)

    def test_malformed_expected_revision_rejects_before_ik(self):
        locked = self.lock()
        request = request_for(locked)
        request["expected_grasp_content_revision"] = ""
        with self.assertRaises(Exception):
            self.node.plan_digital_twin_object_global(request)
        self.assertEqual(self.node.object_global_planning_adapter.ik_calls, 0)

    def test_checked_in_model_backend_mismatch_rejects_before_ik(self):
        locked = self.lock()
        with patch.object(
            self.backend,
            "get_web_model_calibration_revision",
            return_value="sha256:model-does-not-match-backend",
        ):
            result = self.node.plan_digital_twin_object_global(request_for(locked))
        self.assertEqual(result["planner_status"], "REJECTED")
        self.assertEqual(
            result["failure"]["reason"],
            "MODEL_CALIBRATION_REVISION_MISMATCH",
        )
        self.assertEqual(self.node.object_global_planning_adapter.ik_calls, 0)

    def test_matching_authority_plans_and_embeds_self_describing_metadata(self):
        locked = self.lock()
        result = self.node.plan_digital_twin_object_global(request_for(locked))
        self.assertTrue(result["ok"])
        self.assertGreater(self.node.object_global_planning_adapter.ik_calls, 0)
        self.assertEqual(result["grasp"]["source"], "LOCKED OPERATOR GRASP")
        self.assertEqual(result["grasp"]["content_revision"], locked["content_revision"])
        self.assertEqual(result["grasp"]["lock_generation"], locked["lock_generation"])
        self.assertEqual(result["calibration"]["revision"], CALIBRATION_REVISION)
        self.assertEqual(result["calibration"]["revision_status"], "MATCH")
        self.assertEqual(result["waypoints"][0], {
            "identifier": "W0",
            "translation_m": [0.0, 0.0, 0.8],
            "rpy_rad": [0.1, -0.2, 0.3],
            "speed_percent": 100.0,
            "acceleration_percent": 100.0,
            "has_outgoing_segment": True,
        })

    def test_custom_left_right_targets_and_one_locked_grasp_for_all_samples(self):
        plan, _adapter = pure_custom_plan()
        model = ObjectGraspModel(
            RigidTransform.from_translation_rpy(
                locked_payload()["left"]["translation_m"],
                locked_payload()["left"]["rpy_rad"],
            ),
            RigidTransform.from_translation_rpy(
                locked_payload()["right"]["translation_m"],
                locked_payload()["right"]["rpy_rad"],
            ),
        )
        for sample in plan["object_samples"]:
            world_object = RigidTransform.from_matrix(sample["object_pose"]["matrix"])
            targets = model.compute_world_grasp_targets(world_object)
            self.assertTrue(RigidTransform.from_matrix(
                sample["left_target"]["matrix"]
            ).almost_equal(targets.world_T_left))
            self.assertTrue(RigidTransform.from_matrix(
                sample["right_target"]["matrix"]
            ).almost_equal(targets.world_T_right))


class AuthoritativePhase4Tests(unittest.TestCase):
    def test_fingerprint_is_sensitive_to_all_grasp_calibration_identity(self):
        plan, _adapter = pure_custom_plan()
        base = compute_plan_fingerprint(plan)
        mutations = []
        for side, component in (
            ("left", "translation_m"),
            ("left", "rpy_rad"),
            ("right", "translation_m"),
            ("right", "rpy_rad"),
        ):
            changed = deepcopy(plan)
            changed["grasp"][side][component][0] += 0.01
            mutations.append(changed)
        generation = deepcopy(plan)
        generation["grasp"]["lock_generation"] += 1
        mutations.append(generation)
        calibration = deepcopy(plan)
        calibration["calibration"]["revision"] = "sha256:changed"
        mutations.append(calibration)
        waypoint = deepcopy(plan)
        waypoint["waypoints"][1]["translation_m"][0] += 0.01
        mutations.append(waypoint)
        for changed in mutations:
            self.assertNotEqual(base, compute_plan_fingerprint(changed))

    def test_locked_targets_and_fk_relative_pose_validate_against_expected(self):
        plan, _adapter = pure_custom_plan()
        result = validate_phase4_trajectory(
            plan,
            fk_adapter=PlannedTargetFk(plan),
            joint_position_limits=LIMITS,
            start_state={"left": [0.0] * 6, "right": [0.0] * 6, "source": "TEST"},
            dynamics_limits=load_moveit_joint_dynamics_limits(),
        )
        self.assertEqual(result["fixed_grasp"]["status"], "PASS")
        self.assertEqual(result["relative_pose"]["status"], "PASS")
        self.assertEqual(
            result["fixed_grasp"]["grasp_content_revision"],
            plan["grasp"]["content_revision"],
        )

    def test_inconsistent_stored_left_or_right_target_fails_fixed_grasp(self):
        for side in ("left_target", "right_target"):
            plan, _adapter = pure_custom_plan()
            plan["object_samples"][1][side]["matrix"][0][3] += 0.01
            result = validate_phase4_trajectory(
                plan,
                fk_adapter=PlannedTargetFk(plan),
                joint_position_limits=LIMITS,
                start_state={
                    "left": [0.0] * 6,
                    "right": [0.0] * 6,
                    "source": "OFFLINE_TEST",
                },
                dynamics_limits=load_moveit_joint_dynamics_limits(),
            )
            with self.subTest(side=side):
                self.assertEqual(result["fixed_grasp"]["status"], "FAIL")
                self.assertEqual(result["fixed_grasp"]["first_failure"]["sample_index"], 1)

    def test_backend_authority_rejects_old_plan_and_unlocked_current_state(self):
        backend, cleanup = _mocked_backend_import()
        try:
            node = backend.node
            node.rigid_grasp_configuration = AuthoritativeRigidGraspState()
            locked = node.lock_digital_twin_grasp_configuration(locked_payload())[
                "locked_snapshot"
            ]
            plan, _adapter = pure_custom_plan()
            plan["grasp"]["content_revision"] = locked["content_revision"]
            plan["grasp"]["lock_generation"] = locked["lock_generation"]
            plan["grasp"]["lock_revision"] = locked["lock_revision"]
            self.assertEqual(
                node._phase4_authoritative_input_validation(plan)["status"], "PASS"
            )
            old = deepcopy(plan)
            old["grasp"]["lock_generation"] += 1
            self.assertEqual(
                node._phase4_authoritative_input_validation(old)["reason_code"],
                "PLAN_LOCK_GENERATION_STALE",
            )
            old_calibration = deepcopy(plan)
            old_calibration["calibration"]["revision"] = "sha256:old-calibration"
            self.assertEqual(
                node._phase4_authoritative_input_validation(old_calibration)[
                    "reason_code"
                ],
                "PLAN_CALIBRATION_REVISION_STALE",
            )
            old_model = deepcopy(plan)
            old_model["calibration"]["model_revision"] = "sha256:old-model"
            self.assertEqual(
                node._phase4_authoritative_input_validation(old_model)["reason_code"],
                "MODEL_CALIBRATION_REVISION_MISMATCH",
            )
            node.unlock_digital_twin_grasp_configuration()
            self.assertEqual(
                node._phase4_authoritative_input_validation(plan)["reason_code"],
                "CURRENT_GRASP_UNLOCKED",
            )
        finally:
            cleanup()


class AuthoritativeWebIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = (ROOT / "dual_arm_app/web/digital_twin.js").read_text(
            encoding="utf-8"
        )
        cls.planning = (
            ROOT / "dual_arm_app/web/digital_twin_phase3_planning.js"
        ).read_text(encoding="utf-8")
        cls.planner = (
            ROOT / "dual_arm_app/backend/object_global_planning.py"
        ).read_text(encoding="utf-8")

    def test_web_sends_all_expected_revisions_and_rejects_inflight_stale_result(self):
        body = self.controller.split("async function planObjectGlobal", 1)[1].split(
            "function bindObjectWaypointPlanningControls", 1
        )[0]
        for field in (
            "expectedGraspContentRevision",
            "expectedLockGeneration",
            "expectedLockRevision",
            "expectedCalibrationRevision",
            "expectedModelCalibrationRevision",
        ):
            self.assertIn(field, body)
        self.assertIn("samePhase3PlanningAuthority(authorityAtRequest, authorityNow)", body)
        self.assertIn("STALE PLANNING RESPONSE", body)
        self.assertIn("requestGeneration !== objectGlobalPlanState.requestGeneration", body)

    def test_generate_refreshes_backend_grasp_authority_before_planning(self):
        body = self.controller.split("async function planObjectGlobal", 1)[1].split(
            "function bindObjectWaypointPlanningControls", 1
        )[0]
        refresh_index = body.index("requestGraspConfiguration(fetchImpl)")
        request_index = body.index("requestObjectGlobalPlan(request, fetchImpl)")
        self.assertLess(refresh_index, request_index)
        self.assertIn(
            "applyBackendRigidGraspState(await requestGraspConfiguration(fetchImpl))",
            body,
        )
        self.assertIn('objectGlobalPlanState.status = "BLOCKED"', body)
        self.assertIn("Lock the grasp again before Generate Trajectory", body)
        self.assertIn('objectGlobalPlanState.status = "PLANNING"', body)
        status_body = self.controller.split("function updateOperatorPlanStatus", 1)[
            1
        ].split("function updateObjectWaypointPlanningUi", 1)[0]
        lock_prompt = status_body.index('element.textContent = "LOCK GRASP TO CONTINUE"')
        failed_prompt = status_body.index('element.textContent = "FAILED"')
        self.assertLess(lock_prompt, failed_prompt)

    def test_planner_ui_is_integrated_and_not_pending(self):
        self.assertIn('"Generate Trajectory"', self.controller)
        self.assertIn("PRE_P5_C1_C2_INTEGRATED", self.controller)
        self.assertNotIn("CUSTOM_GRASP_PLANNER_INTEGRATION_PENDING", self.controller)

    def test_production_planner_has_no_implicit_synthetic_grasp_authority(self):
        self.assertNotIn("SYNTHETIC_OBJECT_GRASP_FIXTURE", self.planner)
        signature = self.planner.split("def plan_object_global", 1)[1].split(
            ") -> dict[str, Any]:", 1
        )[0]
        self.assertIn("grasp_model: ObjectGraspModel", signature)
        self.assertIn("planning_authority: Any", signature)


if __name__ == "__main__":
    unittest.main()
