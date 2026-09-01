"""Pure offline Phase-4B collision classification and sampling tests."""

from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace
import unittest

from dual_arm_app.backend.moveit_planning_scene_validation import (
    MoveItPlanningSceneValidationAdapter,
)
from dual_arm_app.backend.phase4_collision_validation import (
    ENVIRONMENT_ROBOT,
    INTER_ARM,
    OBJECT_ENVIRONMENT,
    OBJECT_ROBOT,
    OFFLINE_FIXTURE_LABEL,
    OTHER_UNKNOWN,
    SELF_LEFT,
    SELF_RIGHT,
    STATE_INVALID_NO_CONTACT,
    build_phase4b_collision_result,
    build_synchronized_scene_samples,
    canonical_collision_registry,
    classify_collision_pair,
    synthetic_environment_collision_fixture,
    synthetic_object_collision_fixture,
)


IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def plan(end_translation=1.0, end_joint=0.11):
    end_matrix = deepcopy(IDENTITY)
    end_matrix[0][3] = end_translation
    return {
        "ok": True,
        "planner_status": "READY",
        "trajectory_name": "Phase4B_Test",
        "duration_s": 1.0,
        "global_path": [
            {"time_from_start_s": 0.0, "left": [0.0] * 6, "right": [0.0] * 6, "combined": [0.0] * 12},
            {"time_from_start_s": 1.0, "left": [end_joint] * 6, "right": [end_joint] * 6, "combined": [end_joint] * 12},
        ],
        "object_samples": [
            {"time_from_start_s": 0.0, "object_pose": {"matrix": deepcopy(IDENTITY)}},
            {"time_from_start_s": 1.0, "object_pose": {"matrix": end_matrix}},
        ],
    }


def rotating_plan():
    value = plan(end_translation=0.0, end_joint=0.11)
    value["object_samples"][1]["object_pose"]["matrix"] = [
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return value


def point(valid=True, contacts=(), index=0):
    return {
        "point_index": index,
        "time_from_start_s": float(index),
        "valid": valid,
        "collision": not valid and bool(contacts),
        "contacts": [
            {"body_1": body_1, "body_2": body_2, "depth_m": depth}
            for body_1, body_2, depth in contacts
        ],
        "reason": None if valid or contacts else "STATE INVALID — NO COLLISION CONTACT RETURNED",
    }


def robot_validation(stored=None, sampled=None, status="PASS"):
    stored = [point()] if stored is None else stored
    result = {"status": status, "points": stored}
    if sampled is not None:
        result["sampled_path"] = {"status": "PASS" if all(item["valid"] for item in sampled) else "FAIL", "samples": sampled}
    return result


class FakeSceneValidator:
    def __init__(self, contacts_for=None):
        self.contacts_for = contacts_for or (lambda _sample: ())
        self.samples = []
        self.object_scene = None
        self.environment_scene = None

    def validate_scene_samples(self, samples, *, object_scene, environment_scene):
        self.samples = deepcopy(list(samples))
        self.object_scene = deepcopy(object_scene)
        self.environment_scene = deepcopy(list(environment_scene))
        output = []
        for sample in samples:
            contacts = self.contacts_for(sample)
            output.append({
                **deepcopy(dict(sample)),
                "valid": not bool(contacts),
                "contacts": [
                    {"body_1": a, "body_2": b, "depth_m": depth}
                    for a, b, depth in contacts
                ],
            })
        return {"status": "PASS" if all(item["valid"] for item in output) else "FAIL", "samples": output, "error": None}


class CollisionClassificationTests(unittest.TestCase):
    def setUp(self):
        self.registry = canonical_collision_registry(
            object_ids=("phase4b_offline_test_workpiece",),
            environment_ids=("phase4b_offline_test_environment_box",),
            allowed_grasp_contact_links=("left_J6", "right_J6"),
        )

    def assert_category(self, expected, body_1, body_2):
        forward = classify_collision_pair(body_1, body_2, self.registry)
        reverse = classify_collision_pair(body_2, body_1, self.registry)
        self.assertEqual(forward["category"], expected)
        self.assertEqual(reverse["category"], expected)

    def test_self_left_and_self_right_use_explicit_urdf_identities(self):
        self.assert_category(SELF_LEFT, "left_J5", "left_J6")
        self.assert_category(SELF_RIGHT, "right_J5", "right_J6")
        self.assertIn("COMBINED_ROBOT_URDF", self.registry.identity_source)

    def test_known_inter_arm_contact_is_order_invariant(self):
        self.assert_category(INTER_ARM, "left_J6", "right_J2")

    def test_object_robot_environment_and_unknown_categories(self):
        object_id = "phase4b_offline_test_workpiece"
        environment_id = "phase4b_offline_test_environment_box"
        self.assert_category(OBJECT_ROBOT, object_id, "left_J4")
        self.assert_category(ENVIRONMENT_ROBOT, environment_id, "right_J3")
        self.assert_category(OBJECT_ENVIRONMENT, object_id, environment_id)
        self.assert_category(OTHER_UNKNOWN, "mystery_a", "mystery_b")

    def test_only_explicit_grasp_links_are_allowed(self):
        allowed = classify_collision_pair("phase4b_offline_test_workpiece", "left_J6", self.registry)
        disallowed = classify_collision_pair("phase4b_offline_test_workpiece", "left_J5", self.registry)
        self.assertTrue(allowed["allowed_grasp_contact"])
        self.assertFalse(disallowed["allowed_grasp_contact"])


class Phase4CollisionResultTests(unittest.TestCase):
    def test_default_object_and_environment_are_not_configured_not_pass(self):
        result = build_phase4b_collision_result(plan(), robot_validation())
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertEqual(result["object_scene"]["status"], "NOT_CONFIGURED")
        self.assertEqual(result["object_collision"]["status"], "NOT_EVALUATED")
        self.assertEqual(result["environment_scene"]["status"], "NOT_CONFIGURED")
        self.assertEqual(result["environment_collision"]["status"], "NOT_EVALUATED")
        self.assertIn("VISUALIZATION-ONLY", result["object_scene"]["source"])

    def test_invalid_state_without_contact_is_not_collision(self):
        result = build_phase4b_collision_result(
            plan(), robot_validation(stored=[point(False, ())], status="FAIL")
        )
        stored = result["robot_collision"]["stored_points"]["points"][0]
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(stored["state_classification"], STATE_INVALID_NO_CONTACT)
        self.assertEqual(result["robot_collision"]["self_collision"]["status"], "PASS")
        self.assertEqual(result["collision_summary"]["first_failure"], None)

    def test_self_collision_first_failure_and_penetration(self):
        collision = point(False, [("left_J5", "left_J6", 0.002)])
        result = build_phase4b_collision_result(
            plan(), robot_validation(stored=[collision], status="FAIL")
        )
        summary = result["robot_collision"]["self_collision"]
        self.assertEqual(summary["status"], "FAIL")
        self.assertEqual(summary["first_failure"]["category"], SELF_LEFT)
        self.assertEqual(summary["max_penetration_depth_m"], 0.002)

    def test_inter_arm_stored_and_sampled_are_both_classified(self):
        stored = point(False, [("left_J6", "right_J2", 0.003)])
        sampled = [{
            **point(False, [("right_J2", "left_J6", 0.004)]),
            "segment_index": 0, "sample_index": 1, "alpha": 0.5,
            "time_from_start_s": 0.5,
        }]
        result = build_phase4b_collision_result(
            plan(), robot_validation(stored=[stored], sampled=sampled, status="FAIL")
        )
        inter = result["robot_collision"]["inter_arm_collision"]
        self.assertEqual(inter["status"], "FAIL")
        self.assertEqual(inter["collision_count"], 2)
        self.assertEqual(len(inter["collision_pairs"]), 1)

    def test_synchronized_object_pose_uses_same_alpha_and_time(self):
        synchronized = build_synchronized_scene_samples(plan(), 0.05)
        samples = synchronized["sampled_path"]["samples"]
        self.assertEqual(len(samples), 2)
        self.assertAlmostEqual(samples[0]["alpha"], 1 / 3)
        self.assertAlmostEqual(samples[0]["time_from_start_s"], 1 / 3)
        self.assertAlmostEqual(samples[0]["object_pose_matrix"][0][3], 1 / 3)
        self.assertAlmostEqual(samples[1]["object_pose_matrix"][0][3], 2 / 3)

    def test_synchronized_object_pose_slerps_rotation_for_sampled_collision(self):
        synchronized = build_synchronized_scene_samples(rotating_plan(), 0.05)
        samples = synchronized["sampled_path"]["samples"]
        self.assertEqual(len(samples), 2)
        first = samples[0]["object_pose_matrix"]
        second = samples[1]["object_pose_matrix"]
        self.assertAlmostEqual(first[0][0], 0.5, places=9)
        self.assertAlmostEqual(first[1][0], 0.8660254037844386, places=9)
        self.assertAlmostEqual(second[0][0], -0.5, places=9)
        self.assertAlmostEqual(second[1][0], 0.8660254037844386, places=9)

    def test_object_collision_pass_fail_and_allowed_contact(self):
        geometry = synthetic_object_collision_fixture()
        passing = build_phase4b_collision_result(
            plan(), robot_validation(), scene_validator=FakeSceneValidator(), object_geometry=geometry
        )
        self.assertEqual(passing["object_collision"]["status"], "PASS")
        failing = build_phase4b_collision_result(
            plan(), robot_validation(),
            scene_validator=FakeSceneValidator(lambda sample: [
                (geometry["id"], "left_J5", 0.01)
            ] if sample["location_kind"] == "SAMPLED_PATH" else []),
            object_geometry=geometry,
        )
        self.assertEqual(failing["object_collision"]["status"], "FAIL")
        self.assertEqual(failing["object_collision"]["first_failure"]["segment_index"], 0)
        allowed = build_phase4b_collision_result(
            plan(), robot_validation(),
            scene_validator=FakeSceneValidator(lambda _sample: [(geometry["id"], "left_J6", 0.01)]),
            object_geometry=geometry,
        )
        self.assertEqual(allowed["object_collision"]["status"], "PASS")

    def test_environment_configured_empty_is_pass_and_fixture_collision_fails(self):
        environment = synthetic_environment_collision_fixture()
        passing = build_phase4b_collision_result(
            plan(), robot_validation(), scene_validator=FakeSceneValidator(),
            environment_objects=environment,
        )
        self.assertEqual(passing["environment_scene"]["status"], "CONFIGURED")
        self.assertEqual(passing["environment_collision"]["status"], "PASS")
        failing = build_phase4b_collision_result(
            plan(), robot_validation(),
            scene_validator=FakeSceneValidator(lambda _sample: [(environment[0]["id"], "right_J3", 0.006)]),
            environment_objects=environment,
        )
        self.assertEqual(failing["environment_collision"]["status"], "FAIL")
        self.assertEqual(failing["collision_summary"]["first_failure"]["category"], ENVIRONMENT_ROBOT)

    def test_synthetic_fixtures_are_labeled_not_physical(self):
        self.assertEqual(synthetic_object_collision_fixture()["source"], OFFLINE_FIXTURE_LABEL)
        self.assertEqual(synthetic_environment_collision_fixture()[0]["source"], OFFLINE_FIXTURE_LABEL)

    def test_scene_validator_receives_every_moving_pose(self):
        validator = FakeSceneValidator()
        build_phase4b_collision_result(
            plan(), robot_validation(), scene_validator=validator,
            object_geometry=synthetic_object_collision_fixture(),
        )
        translations = [sample["object_pose_matrix"][0][3] for sample in validator.samples]
        self.assertEqual(translations, [0.0, 1.0, 1 / 3, 2 / 3])


class _Pose:
    def __init__(self):
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class _CollisionObject:
    ADD = 0
    REMOVE = 2
    def __init__(self):
        self.header = SimpleNamespace(frame_id="")
        self.pose = _Pose()
        self.id = ""
        self.operation = None
        self.primitives = []
        self.primitive_poses = []


class _Primitive:
    BOX = 1
    def __init__(self): self.type = None; self.dimensions = []


class _Entry:
    def __init__(self): self.enabled = []


class _Matrix:
    def __init__(self): self.entry_names = []; self.entry_values = []


class _Scene:
    def __init__(self):
        self.is_diff = False
        self.world = SimpleNamespace(collision_objects=[])
        self.allowed_collision_matrix = _Matrix()


class _Components:
    WORLD_OBJECT_NAMES = 1
    ALLOWED_COLLISION_MATRIX = 2


class _GetRequest:
    def __init__(self): self.components = SimpleNamespace(components=0)


class _ApplyRequest:
    def __init__(self): self.scene = None


class _ReadyClient:
    def service_is_ready(self): return True


class _StateBridge:
    def validate_trajectory(self, trajectory):
        point_value = trajectory["points"][0]
        return {"status": "PASS", "points": [point(True, (), 0) | {"time_from_start_s": point_value["time_from_start_s"]}]}


class PlanningSceneAdapterTests(unittest.TestCase):
    def test_configured_scene_is_applied_then_removed_and_matrix_restored(self):
        get_client, apply_client = _ReadyClient(), _ReadyClient()
        applied = []
        original = _Scene()

        def call(client, request, _timeout):
            if client is get_client:
                return SimpleNamespace(scene=original)
            applied.append(deepcopy(request.scene))
            return SimpleNamespace(success=True)

        adapter = MoveItPlanningSceneValidationAdapter(
            get_client, apply_client, _StateBridge(), call,
            {
                "AllowedCollisionEntry": _Entry,
                "CollisionObject": _CollisionObject,
                "PlanningScene": _Scene,
                "PlanningSceneComponents": _Components,
                "ApplyRequest": _ApplyRequest,
                "GetRequest": _GetRequest,
                "SolidPrimitive": _Primitive,
            },
        )
        sample = build_synchronized_scene_samples(plan(), 1.0)["stored_samples"][0]
        result = adapter.validate_scene_samples(
            [sample],
            object_scene=synthetic_object_collision_fixture(),
            environment_scene=synthetic_environment_collision_fixture(),
        )
        self.assertEqual(result["status"], "PASS")
        self.assertGreaterEqual(len(applied), 3)
        cleanup = applied[-1]
        self.assertEqual(
            {item.id for item in cleanup.world.collision_objects},
            {"phase4b_offline_test_workpiece", "phase4b_offline_test_environment_box"},
        )
        self.assertTrue(all(item.operation == item.REMOVE for item in cleanup.world.collision_objects))

    def test_preexisting_validation_id_is_rejected_without_mutation(self):
        get_client, apply_client = _ReadyClient(), _ReadyClient()
        original = _Scene()
        existing = _CollisionObject(); existing.id = "phase4b_offline_test_workpiece"
        original.world.collision_objects = [existing]
        applied = []

        def call(client, request, _timeout):
            if client is get_client: return SimpleNamespace(scene=original)
            applied.append(request.scene); return SimpleNamespace(success=True)

        adapter = MoveItPlanningSceneValidationAdapter(
            get_client, apply_client, _StateBridge(), call,
            {"AllowedCollisionEntry": _Entry, "CollisionObject": _CollisionObject,
             "PlanningScene": _Scene, "PlanningSceneComponents": _Components,
             "ApplyRequest": _ApplyRequest, "GetRequest": _GetRequest,
             "SolidPrimitive": _Primitive},
        )
        sample = build_synchronized_scene_samples(plan(), 1.0)["stored_samples"][0]
        result = adapter.validate_scene_samples(
            [sample], object_scene=synthetic_object_collision_fixture(), environment_scene=[]
        )
        self.assertEqual(result["status"], "ERROR")
        self.assertIn("already exists", result["error"])
        self.assertEqual(applied, [])

    def test_cleanup_failure_is_never_silently_reported_as_pass(self):
        get_client, apply_client = _ReadyClient(), _ReadyClient()
        original = _Scene()

        def call(client, request, _timeout):
            if client is get_client:
                return SimpleNamespace(scene=original)
            objects = request.scene.world.collision_objects
            removing = bool(objects) and all(item.operation == item.REMOVE for item in objects)
            return SimpleNamespace(success=not removing)

        adapter = MoveItPlanningSceneValidationAdapter(
            get_client, apply_client, _StateBridge(), call,
            {"AllowedCollisionEntry": _Entry, "CollisionObject": _CollisionObject,
             "PlanningScene": _Scene, "PlanningSceneComponents": _Components,
             "ApplyRequest": _ApplyRequest, "GetRequest": _GetRequest,
             "SolidPrimitive": _Primitive},
        )
        sample = build_synchronized_scene_samples(plan(), 1.0)["stored_samples"][0]
        result = adapter.validate_scene_samples(
            [sample], object_scene=synthetic_object_collision_fixture(), environment_scene=[]
        )
        self.assertEqual(result["status"], "ERROR")
        self.assertIn("cleanup failed", result["error"])


if __name__ == "__main__":
    unittest.main()
