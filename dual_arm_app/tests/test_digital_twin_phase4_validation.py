"""Static Web/backend integration checks for Phase-4A plan-only validation."""

from __future__ import annotations

from html.parser import HTMLParser
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
HTML_PATH = WEB / "index.html"
CONTROLLER_PATH = WEB / "digital_twin.js"
SOURCE_PATH = WEB / "digital_twin_phase4_validation.js"
BACKEND_PATH = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
CORE_PATH = ROOT / "dual_arm_app/backend/phase4_trajectory_validation.py"
FK_PATH = ROOT / "dual_arm_app/backend/moveit_fk_validation.py"


class _IdParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []

    def handle_starttag(self, _tag, attrs):
        self.ids.extend(value for name, value in attrs if name == "id")


class DigitalTwinPhase4ValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")
        cls.backend = BACKEND_PATH.read_text(encoding="utf-8")
        cls.core = CORE_PATH.read_text(encoding="utf-8")
        cls.fk = FK_PATH.read_text(encoding="utf-8")

    def test_panel_contains_required_phase4_readouts_and_warnings(self):
        for text in (
            "Phase 4 — Trajectory / Relative Pose / Dynamics",
            "Common Timeline", "Start Transition", "End State",
            "FK Target Validation", "Relative Translation Error [m]",
            "Relative Orientation Error [rad]", "Fixed Grasp Relationship",
            "Maximum Joint Velocity [rad/s]", "Velocity Limit Source",
            "Maximum Discrete Acceleration [rad/s²]",
            "Acceleration Limit Status", "Acceleration Limit Validation",
            "MODEL-CONSISTENCY CHECK", "NOT PHYSICAL METROLOGY",
            "ACCELERATION LIMIT UNAVAILABLE", "NOT EXECUTION READY",
        ):
            with self.subTest(text=text):
                self.assertIn(text, self.html)

    def test_no_phase4_execute_button_exists(self):
        phase4_html = self.html.split("Phase 4 —", 1)[1].split("</div>\n      </div>", 1)[0]
        self.assertNotIn(">Execute<", phase4_html)
        self.assertNotIn("digitalTwinPhase4Execute", self.html)

    def test_start_state_button_is_explicit_read_only_capture(self):
        self.assertIn("Use Current Actual Joints as Validation Start", self.html)
        self.assertIn("useCurrentActualJointsAsPhase4ValidationStart", self.controller)
        self.assertIn('source: "LIVE_ACTUAL_MIRROR"', self.controller)
        self.assertIn('startStateKind: "UNAVAILABLE"', self.controller)

    def test_public_inspection_api_has_get_validate_set_clear(self):
        for name in (
            "getPhase4TrajectoryValidationState",
            "validateCurrentGlobalPlanPhase4",
            "setPhase4ValidationStartState",
            "clearPhase4ValidationStartState",
        ):
            self.assertIn(name, self.controller)

    def test_web_request_uses_current_successful_global_plan_and_optional_start(self):
        self.assertIn("plan: objectGlobalPlanState.plan", self.controller)
        self.assertIn("start_state: startState === null", self.source)
        self.assertIn("A successful Phase-3 Global Plan is required", self.source)

    def test_endpoint_is_registered_as_no_motion(self):
        endpoint = "/api/digital-twin/validate-phase4-trajectory"
        self.assertGreaterEqual(self.backend.count(endpoint), 2)
        no_motion_slice = self.backend.split("_D33_NO_MOTION_PATHS", 1)[1].split(")", 1)[0]
        self.assertIn(endpoint, no_motion_slice)

    def test_phase4_production_files_have_no_motion_api_reference(self):
        production = "\n".join((self.source, self.core, self.fk))
        for forbidden in (
            "/api/jog", "/api/home", "/api/direct", "/api/waypoint/run",
            "/api/program/run", "/api/sequence/run", "joint_move",
            "linear_move", "servo",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, production)

    def test_moveit_fk_not_jaka_getfk_and_same_phase3_tip_constants(self):
        self.assertIn('COMPUTE_FK_SERVICE = "/compute_fk"', self.fk)
        self.assertIn("LEFT_IK_LINK_NAME", self.fk)
        self.assertIn("RIGHT_IK_LINK_NAME", self.fk)
        self.assertNotIn("jaka_msgs", self.fk)
        self.assertNotIn("from jaka_msgs.srv import", self.fk)

    def test_html_ids_remain_unique(self):
        parser = _IdParser()
        parser.feed(self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))

    def test_existing_phase1_to_phase3_apis_and_validation_are_preserved(self):
        for name in (
            "validateLoadedTrajectory", "validateLoadedTrajectoryJointLimits",
            "planObjectGlobal", "getObjectGlobalPlanState",
            "getIntegratedPlanPreviewState", "getObjectGraspRelativeState",
        ):
            self.assertIn(name, self.controller)


if __name__ == "__main__":
    unittest.main()
