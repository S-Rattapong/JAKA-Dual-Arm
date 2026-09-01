"""Feature 2 regressions: Center waypoint editing UX is NO MOTION."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]
PLANNING = ROOT / "dual_arm_app/web/digital_twin_phase3_planning.js"
CONTROLLER = ROOT / "dual_arm_app/web/digital_twin.js"
HTML = ROOT / "dual_arm_app/web/index.html"


def run_node(script: str) -> dict:
    result = subprocess.run(
        ["node", "--input-type=module", "-e", script],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(result.stdout)


class Feature2PureWaypointTests(unittest.TestCase):
    def test_auto_renumber_duplicate_and_drag_reorder(self):
        payload = run_node(r'''
import {
  renumberObjectPlanningWaypoints,
  duplicateObjectPlanningWaypoint,
  reorderObjectPlanningWaypoint,
} from "./dual_arm_app/web/digital_twin_phase3_planning.js";
const p = (id, x) => ({identifier:id, translation_m:[x,0,0], rpy_rad:[0,0,0]});
const base = renumberObjectPlanningWaypoints([p("alpha",0),p("z",1),p("q",2)]);
const duplicated = duplicateObjectPlanningWaypoint(base, 1);
const reordered = reorderObjectPlanningWaypoint(duplicated, 3, 1);
console.log(JSON.stringify({base,duplicated,reordered}));
''')
        self.assertEqual([w["identifier"] for w in payload["base"]], ["W0","W1","W2"])
        self.assertEqual([w["identifier"] for w in payload["duplicated"]], ["W0","W1","W2","W3"])
        self.assertEqual(payload["duplicated"][1]["translation_m"], payload["duplicated"][2]["translation_m"])
        self.assertEqual([w["translation_m"][0] for w in payload["reordered"]], [0,2,1,1])
        self.assertEqual([w["identifier"] for w in payload["reordered"]], ["W0","W1","W2","W3"])

    def test_reorder_noop_and_bounds_are_safe(self):
        payload = run_node(r'''
import {reorderObjectPlanningWaypoint} from "./dual_arm_app/web/digital_twin_phase3_planning.js";
const p = (x) => ({identifier:`old${x}`, translation_m:[x,0,0], rpy_rad:[0,0,0]});
const base = [p(0),p(1),p(2)];
const same = reorderObjectPlanningWaypoint(base, 1, 2);
let error = null;
try { reorderObjectPlanningWaypoint(base, 5, 0); } catch (e) { error = e.message; }
console.log(JSON.stringify({same,error}));
''')
        self.assertEqual([w["translation_m"][0] for w in payload["same"]], [0,1,2])
        self.assertIn("out of range", payload["error"])


class Feature2UiContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = CONTROLLER.read_text(encoding="utf-8")
        cls.html = HTML.read_text(encoding="utf-8")

    def test_rows_offer_drag_duplicate_move_center_and_delete(self):
        body = self.controller.split("function renderObjectWaypointRows", 1)[1].split(
            "function objectWaypointFromAddControls", 1
        )[0]
        for token in (
            'dragHandle.draggable = true',
            'event.dataTransfer.setData("text/plain", String(index))',
            "reorderObjectPlanningWaypointInState",
            '"Move Center Here"',
            '"Duplicate"',
            '"Delete"',
            "idInput.readOnly = true",
        ):
            self.assertIn(token, body)
        self.assertNotIn('moveObjectPlanningWaypointInState(index, -1)', body)
        self.assertNotIn('moveObjectPlanningWaypointInState(index, 1)', body)

    def test_identifiers_are_auto_numbered_every_time_state_changes(self):
        self.assertIn(
            "objectWaypointPlanningState.waypoints = renumberObjectPlanningWaypoints(waypoints)",
            self.controller,
        )
        self.assertIn('nextIdentifierInput.value = `W${state.waypoints.length}`', self.controller)
        self.assertIn("nextIdentifierInput.readOnly = true", self.controller)
        self.assertIn("Waypoint identifiers are numbered automatically", self.controller)

    def test_move_center_here_is_visual_only_no_motion(self):
        body = self.controller.split("function moveCenterPreviewToObjectWaypoint", 1)[1].split(
            "function clearObjectPlanningWaypoints", 1
        )[0]
        self.assertIn("applyObjectPreviewPose", body)
        self.assertIn("GRASP_SELECTED_FRAMES.CENTER", body)
        self.assertIn("pauseObjectTrajectoryForManualPreview", body)
        for forbidden in (
            "fetch(", "/api/", "servo_j", "joint_move", "linear_move",
            "motion_abort", "execute_joint_trajectory",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, body)

    def test_step2_headers_and_drag_styling_match_new_actions(self):
        for expected in (
            "<strong>Center</strong><strong>Duplicate</strong><strong>Delete</strong>",
            ".digital-twin-waypoint-drag-handle",
            ".digital-twin-object-waypoint-row.drop-before",
            ".digital-twin-object-waypoint-row.drop-after",
        ):
            self.assertIn(expected, self.html)
        self.assertNotIn("<strong>Up</strong><strong>Down</strong><strong>Delete</strong>", self.html)


if __name__ == "__main__":
    unittest.main()
