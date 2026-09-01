"""Feature 3: hover a Center waypoint to show debug-only frame ghosts."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
HTML = (WEB / "index.html").read_text(encoding="utf-8")
JS = (WEB / "digital_twin.js").read_text(encoding="utf-8")


class WaypointFrameHoverTests(unittest.TestCase):
    def test_ui_explains_hover_debug_preview(self):
        self.assertIn("digitalTwinObjectWaypointHoverPreview", HTML)
        self.assertIn(
            "Hover any waypoint row to preview its Center, Left, and Right Frame Ghosts",
            HTML,
        )

    def test_hover_frames_are_separate_debug_scene_objects(self):
        for token in (
            "Center Waypoint Hover Frame Ghosts — DEBUG ONLY",
            "Hovered Center Frame Ghost",
            "Hovered Left Frame Ghost",
            "Hovered Right Frame Ghost",
            "createWaypointHoverGhostFrame",
        ):
            self.assertIn(token, JS)
        self.assertIn("transparent: true, opacity: 0.42", JS)
        self.assertIn("axes.material.opacity = 0.72", JS)

    def test_hover_pose_uses_waypoint_center_and_locked_grasp_relations(self):
        body = JS.split("function showObjectWaypointHoverPreview", 1)[1].split(
            "function createObjectGraspPreview", 1
        )[0]
        self.assertIn("objectWaypointPlanningState.waypoints[index]", body)
        self.assertIn("currentRigidGraspContent()", body)
        self.assertIn("matrix4FromTranslationRpy(pose)", body)
        self.assertIn("computeWorldGraspFrameMatrices(pose, graspContent)", body)
        self.assertIn("transforms.worldTLeft", body)
        self.assertIn("transforms.worldTRight", body)
        self.assertIn("applyMatrixToFrame(objectWaypointHoverFrames.center", body)

    def test_rows_show_on_enter_hide_on_leave_and_hide_for_drag(self):
        rows = JS.split("function renderObjectWaypointRows", 1)[1].split(
            "function objectWaypointFromAddControls", 1
        )[0]
        self.assertIn('row.addEventListener("pointerenter"', rows)
        self.assertIn("showObjectWaypointHoverPreview(index)", rows)
        self.assertIn('row.addEventListener("pointerleave"', rows)
        self.assertIn("hideObjectWaypointHoverPreview()", rows)
        drag = rows.split('dragHandle.addEventListener("dragstart"', 1)[1]
        self.assertIn("hideObjectWaypointHoverPreview()", drag)

    def test_hover_is_visual_only_and_never_changes_robot_or_path(self):
        body = JS.split("function showObjectWaypointHoverPreview", 1)[1].split(
            "function createObjectGraspPreview", 1
        )[0]
        for forbidden in (
            "setObjectPlanningWaypoints(",
            "applyObjectPreviewPose(",
            "setJointValues(",
            "applyMirrorPoseToMainModel(",
            "servo_j",
            "fetch(",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, body)


if __name__ == "__main__":
    unittest.main()
