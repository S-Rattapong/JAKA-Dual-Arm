"""Static backend/Web safety contracts for Phase-4B collision completion."""

from __future__ import annotations

from html.parser import HTMLParser
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "dual_arm_app/backend/phase4_collision_validation.py"
ADAPTER = ROOT / "dual_arm_app/backend/moveit_planning_scene_validation.py"
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
CONTROLLER = ROOT / "dual_arm_app/web/digital_twin.js"
SOURCE = ROOT / "dual_arm_app/web/digital_twin_phase4_collision.js"
HTML = ROOT / "dual_arm_app/web/index.html"


class IdParser(HTMLParser):
    def __init__(self): super().__init__(); self.ids = []
    def handle_starttag(self, _tag, attrs): self.ids.extend(v for k, v in attrs if k == "id")


class DigitalTwinPhase4CollisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.core = CORE.read_text(encoding="utf-8")
        cls.adapter = ADAPTER.read_text(encoding="utf-8")
        cls.backend = BACKEND.read_text(encoding="utf-8")
        cls.controller = CONTROLLER.read_text(encoding="utf-8")
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.html = HTML.read_text(encoding="utf-8")

    def test_existing_bridge_and_sampler_are_reused(self):
        self.assertIn("generate_trajectory_interior_samples", self.core)
        self.assertIn("self.moveit_state_validation_bridge.validate_trajectory", self.backend)
        self.assertNotIn("GetStateValidity", self.core)

    def test_planning_scene_adapter_uses_existing_node_and_reversible_services(self):
        self.assertIn("from_node", self.adapter)
        self.assertIn('"/get_planning_scene"', self.adapter)
        self.assertIn('"/apply_planning_scene"', self.adapter)
        self.assertIn("_remove_scene", self.adapter)
        self.assertIn("original_matrix", self.adapter)
        self.assertNotIn("rclpy.init", self.adapter)

    def test_endpoint_is_plan_only_and_registered_no_motion(self):
        endpoint = "/api/digital-twin/validate-phase4-collision"
        self.assertGreaterEqual(self.backend.count(endpoint), 2)
        no_motion = self.backend.split("_D33_NO_MOTION_PATHS", 1)[1].split(")", 1)[0]
        self.assertIn(endpoint, no_motion)

    def test_ui_has_required_states_warnings_and_no_execute(self):
        for text in (
            "Phase 4B — Planning Scene / Collision Validation",
            "Robot Stored-Point Collision", "Robot Sampled-Path Collision",
            "Self Collision", "Inter-Arm Collision", "Object Scene",
            "Object Collision", "Environment Scene", "Environment Collision",
            "First Collision Category", "First Collision Pair",
            "OBJECT COLLISION GEOMETRY NOT CONFIGURED",
            "ENVIRONMENT COLLISION SCENE NOT CONFIGURED",
            "NOT CONTINUOUS COLLISION GUARANTEE", "NOT EXECUTION READY",
        ):
            self.assertIn(text, self.html)
        self.assertNotIn("digitalTwinPhase4BExecute", self.html)

    def test_public_state_validate_and_clear_api(self):
        for name in (
            "getPhase4CollisionValidationState",
            "validateCurrentGlobalPlanPhase4Collision",
            "clearPhase4CollisionValidation",
        ):
            self.assertIn(name, self.controller)
        self.assertIn("objectGlobalPlanState.plan", self.controller)

    def test_web_transport_has_only_collision_validation_endpoint(self):
        self.assertIn("PHASE4B_COLLISION_ENDPOINT", self.source)
        self.assertIn('method: "POST"', self.source)
        self.assertNotIn("execute", self.source.lower())

    def test_new_phase4b_production_has_zero_motion_references(self):
        production = "\n".join((self.core, self.adapter, self.source))
        for forbidden in (
            "/api/jog", "/api/home", "/api/direct", "/api/waypoint/run",
            "/api/program/run", "/api/sequence/run", "joint_move",
            "linear_move", "servo",
        ):
            self.assertNotIn(forbidden, production)

    def test_html_ids_are_unique_and_phase4a_remains(self):
        parser = IdParser(); parser.feed(self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))
        self.assertIn("Phase 4 — Trajectory / Relative Pose / Dynamics", self.html)
        self.assertIn("validateCurrentGlobalPlanPhase4", self.controller)


if __name__ == "__main__":
    unittest.main()
