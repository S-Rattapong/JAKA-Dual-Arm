from __future__ import annotations

from html.parser import HTMLParser
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
HTML = (ROOT / "dual_arm_app/web/index.html").read_text(encoding="utf-8")
VIEWER = (ROOT / "dual_arm_app/web/digital_twin_experiment.js").read_text(encoding="utf-8")
BACKEND = (ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py").read_text(encoding="utf-8")


class Exp2WebTests(unittest.TestCase):
    def test_exp2_panel_controls_metrics_and_series_exist(self):
        for control_id in (
            "experimentExp2Heading", "experimentExp2Analyze", "experimentExp2Status",
            "experimentExp2ActualSource", "experimentExp2Coverage",
            "experimentExp2TranslationRms", "experimentExp2TranslationMax",
            "experimentExp2TranslationWorstTime", "experimentExp2OrientationRms",
            "experimentExp2OrientationMax", "experimentExp2OrientationWorstTime",
            "experimentExp2CenterTranslationRms", "experimentExp2CenterTranslationMax",
            "experimentExp2CenterOrientationRms", "experimentExp2CenterOrientationMax",
            "experimentExp2AxisRows", "experimentExp2Series", "experimentExp2SeriesCanvas",
        ):
            with self.subTest(control_id=control_id):
                self.assertIn(f'id="{control_id}"', HTML)
        self.assertIn("EXP-2 Rigid-Grasp Preservation", HTML)
        self.assertIn("Analyze/Reanalyze Selected Run — NO MOTION", HTML)
        self.assertIn("not external metrology", HTML.lower())
        self.assertIn("never averaged", HTML)

    def test_web_renders_persisted_exp2_and_does_not_recompute_metrics_from_fk(self):
        self.assertIn("renderExp2Analysis(state.loaded.exp2_analysis || null)", VIEWER)
        self.assertIn("state.loaded.exp2_analysis = payload.exp2_analysis", VIEWER)
        self.assertIn("/exp2-analysis", VIEWER)
        self.assertIn("relative_translation?.magnitude_m", VIEWER)
        self.assertIn("center_consistency?.translation_magnitude_m", VIEWER)
        self.assertIn("center_consistency?.orientation_angle", VIEWER)
        self.assertIn("relative_orientation?.angle", VIEWER)
        self.assertIn("usable_synchronized_samples", VIEWER)
        self.assertIn("translation_magnitude", VIEWER)
        self.assertIn("center_orientation", VIEWER)

    def test_center_paths_remain_distinct_and_no_average_is_introduced(self):
        self.assertIn("actualCenterFromLeft", VIEWER)
        self.assertIn("actualCenterFromRight", VIEWER)
        self.assertNotIn("averageCenter", VIEWER)
        self.assertNotIn("averagedCenter", VIEWER)
        self.assertIn("Center-from-Left — magenta", HTML)
        self.assertIn("Center-from-Right — red", HTML)

    def test_existing_custom_bounded_zoom_v2_contract_is_preserved(self):
        self.assertIn("state.controls.enableZoom = false", VIEWER)
        self.assertIn("handleViewerWheel", VIEWER)
        self.assertIn("Math.tanh(deltaPixels / 120)", VIEWER)
        self.assertIn("distance * scale", VIEWER)
        self.assertIn("radius * 30, distance * 8", VIEWER)

    def test_exp2_route_is_experiment_only_and_no_motion_prefix_covers_it(self):
        self.assertIn('@app.post("/api/experiments/{run_id}/exp2-analysis")', BACKEND)
        self.assertIn("analyze_experiment_run_exp2", BACKEND)
        self.assertIn('"/api/experiments",', BACKEND)
        self.assertNotRegex(
            VIEWER,
            r'fetch\s*\(\s*["\']/api/(?:jog|stop|home|direct|program|waypoint/run|digital-twin/phase5)',
        )


if __name__ == "__main__":
    unittest.main()
