"""Static no-motion acceptance checks for the EXP-0 browser viewer."""

from __future__ import annotations

import ast
from collections import Counter
from html.parser import HTMLParser
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[2]
HTML = ROOT / "dual_arm_app/web/index.html"
VIEWER = ROOT / "dual_arm_app/web/digital_twin_experiment.js"
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"


class IdParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []

    def handle_starttag(self, _tag, attrs):
        self.ids.extend(value for name, value in attrs if name == "id")


class ExperimentPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML.read_text(encoding="utf-8")
        cls.viewer = VIEWER.read_text(encoding="utf-8")
        cls.backend = BACKEND.read_text(encoding="utf-8")

    def test_panel_has_recorder_and_complete_run_library_controls(self):
        panel = self.html.split('id="digitalTwinExperimentPanel"', 1)[1].split("</details>", 1)[0]
        self.assertIn("Experimental Validation — EXP-0", panel)
        for control_id in (
            "experimentLabel", "experimentPathType", "experimentFixtureCondition",
            "experimentNotes", "experimentArmRecorder", "experimentDisarmRecorder",
            "experimentRunSelect", "experimentRefreshRuns", "experimentLoadRun",
            "experimentRelabelRun", "experimentDeleteRun", "experimentExportRun",
            "experimentRecorderState", "experimentArmedArtifact",
            "experimentActiveTrajectory", "experimentSampleCounts",
            "experimentLeftTorqueAvailability", "experimentRightTorqueAvailability",
            "experimentRecorderError",
        ):
            with self.subTest(control_id=control_id):
                self.assertIn(f'id="{control_id}"', panel)
        self.assertIn("Arm Recorder — NO MOTION", panel)
        self.assertIn("Disarm — NO MOTION", panel)

    def test_viewer_controls_presets_toggles_slider_and_series_are_present(self):
        for control_id in (
            "experimentPathViewer", "experimentTogglePlanned", "experimentToggleActual",
            "experimentToggleLeft", "experimentToggleRight", "experimentToggleCenter",
            "experimentView3D", "experimentViewTopXY", "experimentViewFrontXZ",
            "experimentViewSideYZ", "experimentFitAll", "experimentSampleSlider",
            "experimentSeriesSide", "experimentSeriesJoint", "experimentTorqueToggle",
            "experimentJointSeries",
        ):
            with self.subTest(control_id=control_id):
                self.assertIn(f'id="{control_id}"', self.html)
        for label in ("3D", "Top XY", "Front XZ", "Side YZ", "Fit All"):
            self.assertIn(f">{label}</button>", self.html)
        self.assertIn("EXP-2 relative pose is shown in its dedicated panel", self.html)
        self.assertIn('state.controls.enableZoom = false', self.viewer)
        self.assertIn('handleViewerWheel', self.viewer)
        self.assertIn('Math.tanh(deltaPixels / 120)', self.viewer)
        self.assertIn('state.controls.maxDistance = Math.max(2, radius * 30, distance * 8)', self.viewer)

    def test_viewer_zoom_is_bounded_and_clipping_tracks_camera_distance(self):
        self.assertIn("state.controls.enableZoom = false", self.viewer)
        self.assertIn("function handleViewerWheel", self.viewer)
        self.assertIn("Math.exp(normalized * 0.035)", self.viewer)
        self.assertIn("state.controls.minDistance", self.viewer)
        self.assertIn("state.controls.maxDistance", self.viewer)
        self.assertIn('addEventListener("change", syncViewerClipping)', self.viewer)
        self.assertIn("function viewerFitDistance", self.viewer)
        self.assertIn("function configureViewerNavigation", self.viewer)
        self.assertIn("function syncViewerClipping", self.viewer)
        self.assertIn("distance / 500", self.viewer)
        self.assertIn("state.controls.maxDistance * 1.5", self.viewer)

    def test_viewer_uses_hidden_urdf_fk_and_never_claims_external_metrology(self):
        self.assertIn('EXPERIMENT_MODEL_URL = "/digital-twin/assets/dual_jaka_a12_web.urdf"', self.viewer)
        self.assertIn('robot.visible = false', self.viewer)
        self.assertIn('left_J6', self.viewer)
        self.assertIn('right_J6', self.viewer)
        self.assertIn('`left_joint_${i + 1}`', self.viewer)
        self.assertIn('`right_joint_${i + 1}`', self.viewer)
        self.assertIn("length: 6", self.viewer)
        self.assertIn("NOT EXTERNAL METROLOGY", self.viewer)
        self.assertIn("recorded encoder/model joints", self.html)

    def test_center_from_left_and_right_are_separate_paths(self):
        self.assertIn("actualCenterFromLeft", self.viewer)
        self.assertIn("actualCenterFromRight", self.viewer)
        self.assertIn("CENTER-FROM-LEFT: W_T_L * inverse(O_T_L)", self.viewer)
        self.assertIn("CENTER-FROM-RIGHT: W_T_R * inverse(O_T_R)", self.viewer)
        self.assertIn("R = Rz(yaw) * Ry(pitch) * Rx(roll)", self.viewer)
        self.assertIn("cy * cp", self.viewer)
        self.assertIn("-sp, cp * sr, cp * cr", self.viewer)
        self.assertNotIn("averageCenter", self.viewer)
        self.assertIn("Center-from-Left — magenta", self.html)
        self.assertIn("Center-from-Right — red", self.html)

    def test_web_module_has_only_experiment_api_requests(self):
        api_literals = set(re.findall(r'["\'](/api/[^"\']+)["\']', self.viewer))
        self.assertEqual(api_literals, {"/api/experiments"})
        self.assertNotIn("window.dualArmDigitalTwin.execute", self.viewer)
        self.assertNotRegex(
            self.viewer,
            r'fetch\s*\(\s*["\']/api/(?:jog|stop|home|direct|program|waypoint/run|digital-twin/phase5)',
        )
        self.assertIn("requestExperiment", self.viewer)
        self.assertIn("sample?.commanded?.[side]?.time_from_start_s", self.viewer)
        self.assertIn("observedTimeS", self.viewer)

    def test_experiment_routes_have_no_motion_calls_or_authority_mutation(self):
        tree = ast.parse(self.backend)
        methods = {
            node.name: ast.unparse(node)
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name.startswith("api_experiment_")
        }
        self.assertEqual(
            set(methods),
            {
                "api_experiment_runs", "api_experiment_recorder_state",
                "api_experiment_arm", "api_experiment_disarm", "api_experiment_load",
                "api_experiment_export", "api_experiment_relabel", "api_experiment_delete",
                "api_experiment_exp1_analysis", "api_experiment_exp2_analysis",
            },
        )
        combined = "\n".join(methods.values())
        for forbidden in (
            "move_both_joint", "request_stop_all", "phase5_execution_coordinator.execute",
            "left_jog", "right_jog", "servo_j", "GetIK", "GetFK",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, combined)
        self.assertIn('"/api/experiments"', self.backend)

    def test_html_ids_remain_unique_and_module_is_loaded_once(self):
        parser = IdParser()
        parser.feed(self.html)
        duplicates = [name for name, count in Counter(parser.ids).items() if count != 1]
        self.assertEqual(duplicates, [])
        self.assertEqual(self.html.count("/web-assets/digital_twin_experiment.js"), 1)


if __name__ == "__main__":
    unittest.main()
