"""Static no-motion UI contracts for EXP-1 trajectory tracking."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
HTML = (ROOT / "dual_arm_app/web/index.html").read_text(encoding="utf-8")
VIEWER = (ROOT / "dual_arm_app/web/digital_twin_experiment.js").read_text(encoding="utf-8")
BACKEND = (ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py").read_text(encoding="utf-8")
ANALYSIS = (ROOT / "dual_arm_app/backend/experimental_trajectory_tracking.py").read_text(encoding="utf-8")


class Exp1WebContracts(unittest.TestCase):
    def test_exp1_controls_summaries_table_and_graph_are_present(self):
        panel = HTML.split('id="digitalTwinExperimentPanel"', 1)[1].split("</details>", 1)[0]
        self.assertIn("EXP-1 Trajectory Tracking Accuracy", panel)
        self.assertIn("Analyze/Reanalyze Selected Run — NO MOTION", panel)
        for control_id in (
            "experimentExp1Analyze", "experimentExp1Status", "experimentExp1ActualSource",
            "experimentExp1CombinedRmse", "experimentExp1CombinedMae", "experimentExp1CombinedMax",
            "experimentExp1CombinedWorst", "experimentExp1CombinedCoverage",
            "experimentExp1LeftRmse", "experimentExp1LeftMae", "experimentExp1LeftMax",
            "experimentExp1LeftWorst", "experimentExp1LeftCoverage",
            "experimentExp1RightRmse", "experimentExp1RightMae", "experimentExp1RightMax",
            "experimentExp1RightWorst", "experimentExp1RightCoverage",
            "experimentExp1JointRows", "experimentExp1ErrorSeries",
        ):
            with self.subTest(control_id=control_id):
                self.assertIn(f'id="{control_id}"', panel)
        for heading in ("Bias [rad]", "MAE [rad]", "RMSE [rad]", "Max |Error| [rad]", "Samples"):
            self.assertIn(heading, panel)

    def test_semantics_are_explicit_and_existing_selectors_drive_error_graph(self):
        self.assertIn("Actual joint [rad] − receive-time-interpolated Planned joint [rad]", HTML)
        self.assertIn("not external metrology", HTML)
        self.assertIn("excluded from the primary metric", HTML)
        self.assertIn("experimentSeriesSide", VIEWER)
        self.assertIn("experimentSeriesJoint", VIEWER)
        self.assertIn("drawExp1ErrorSeries", VIEWER)
        self.assertIn("sample.error_joints_rad[joint]", VIEWER)
        self.assertIn("error_samples", VIEWER)
        self.assertIn("state.loaded.exp1_analysis", VIEWER)
        self.assertIn("/exp1-analysis", VIEWER)

    def test_analysis_and_exp1_route_have_no_motion_calls(self):
        for forbidden in (
            "servo_j", "trajectory submission", "move_both_joint",
            "request_stop_all", "driver restart", "phase5_execution_coordinator.execute",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, ANALYSIS.lower())
        tree = ast.parse(BACKEND)
        route = next(
            node for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
            and node.name == "api_experiment_exp1_analysis"
        )
        route_source = ast.unparse(route)
        self.assertIn("analyze_experiment_run_exp1", route_source)
        for forbidden in ("execute", "servo_j", "stop", "jog", "home"):
            self.assertNotIn(forbidden, route_source.lower())


if __name__ == "__main__":
    unittest.main()
