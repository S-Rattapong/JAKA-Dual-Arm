"""Focused Feature 6 operator-workflow markup and safety contracts."""

from __future__ import annotations

from html import unescape
from html.parser import HTMLParser
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[2]
HTML_PATH = ROOT / "dual_arm_app/web/index.html"


class WorkflowParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.stack: list[tuple[str, str | None]] = []
        self.tag_by_id: dict[str, str] = {}
        self.attrs_by_id: dict[str, dict[str, str | None]] = {}
        self.ancestors_by_id: dict[str, tuple[str, ...]] = {}

    def handle_starttag(self, tag, attrs) -> None:
        attributes = dict(attrs)
        element_id = attributes.get("id")
        self.stack.append((tag, element_id))
        if element_id:
            self.tag_by_id[element_id] = tag
            self.attrs_by_id[element_id] = attributes
            self.ancestors_by_id[element_id] = tuple(
                ancestor_id
                for _, ancestor_id in self.stack[:-1]
                if ancestor_id is not None
            )

    def handle_startendtag(self, tag, attrs) -> None:
        self.handle_starttag(tag, attrs)
        self.stack.pop()

    def handle_endtag(self, tag) -> None:
        for index in range(len(self.stack) - 1, -1, -1):
            if self.stack[index][0] == tag:
                del self.stack[index:]
                return


class Feature6OperatorWorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.parser = WorkflowParser()
        cls.parser.feed(cls.html)
        cls.ids = re.findall(r'id="([^"]+)"', cls.html)
        cls.phase5 = cls.html.split(
            'id="digitalTwinPhase5ExecutionSection"', 1
        )[1].split("</section>", 1)[0]

    def test_guide_has_exactly_three_main_labels(self) -> None:
        guide = re.search(
            r'<nav class="digital-twin-workflow-guide"[^>]*>(.*?)</nav>',
            self.html,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(guide)
        labels = [
            unescape(re.sub(r"<[^>]+>", "", label)).strip()
            for label in re.findall(r"<span>(.*?)</span>", guide.group(1), re.DOTALL)
        ]
        self.assertEqual(
            labels,
            ["1. Grasp Setup", "2. Center Waypoints", "3. Preview & Execute"],
        )

    def test_phase5_is_visible_step3_preview_and_execute(self) -> None:
        attrs = self.parser.attrs_by_id["digitalTwinPhase5ExecutionSection"]
        self.assertEqual(self.parser.tag_by_id["digitalTwinPhase5ExecutionSection"], "section")
        self.assertIn("digital-twin-workflow-stage", attrs["class"].split())
        self.assertEqual(attrs["aria-label"], "Step 3 Preview and Execute")
        self.assertRegex(self.phase5, r"digital-twin-stage-kicker[^>]*>STEP 3</span>")
        self.assertRegex(self.phase5, r"<h3>Preview &amp; Execute</h3>")
        self.assertIn("REAL ROBOT MOTION", self.phase5)

    def test_manual_plan_and_validation_are_collapsed_details(self) -> None:
        expected_children = {
            "digitalTwinOperatorPlanStage": (
                "digitalTwinPlanObjectGlobal",
                "digitalTwinGlobalPlanPlay",
                "digitalTwinGlobalPlanPause",
                "digitalTwinGlobalPlanReset",
                "digitalTwinGlobalPlanScrubber",
                "digitalTwinGlobalPlanCandidateAttemptsInput",
                "digitalTwinObjectWaypointSegmentDuration",
                "digitalTwinObjectWaypointSamplesPerSegment",
                "digitalTwinGlobalPlanState",
                "digitalTwinGlobalPlanError",
            ),
            "digitalTwinOperatorValidateStage": (
                "digitalTwinValidatePhase4Unified",
            ),
        }
        for details_id, child_ids in expected_children.items():
            with self.subTest(details_id=details_id):
                self.assertEqual(self.parser.tag_by_id[details_id], "details")
                self.assertNotIn("open", self.parser.attrs_by_id[details_id])
                for child_id in child_ids:
                    self.assertIn(details_id, self.parser.ancestors_by_id[child_id])

    def test_existing_concise_state_ids_are_unique_and_in_step3(self) -> None:
        for element_id in (
            "digitalTwinOperatorPlanStatus",
            "digitalTwinOperatorValidationStatus",
            "digitalTwinOperatorExecutionReady",
            "digitalTwinOperatorValidationReason",
        ):
            with self.subTest(element_id=element_id):
                self.assertEqual(self.ids.count(element_id), 1)
                self.assertIn(
                    "digitalTwinPhase5ExecutionSection",
                    self.parser.ancestors_by_id[element_id],
                )

    def test_phase5_primary_controls_and_replan_first_are_preserved(self) -> None:
        controls = (
            "digitalTwinPhase5ReplanFromCurrent",
            "digitalTwinPhase5PreviewPlay",
            "digitalTwinPhase5MoveToInitial",
            "digitalTwinPhase5Stop",
            "digitalTwinPhase5Prepare",
        )
        positions = []
        for element_id in controls:
            self.assertIn(f'id="{element_id}"', self.phase5)
            positions.append(self.phase5.index(f'id="{element_id}"'))
        self.assertEqual(positions, sorted(positions))
        self.assertIn("Replan From Current — NO MOTION", self.phase5)
        self.assertIn("Play Frozen Ghost — NO MOTION", self.phase5)
        self.assertIn("Move to Initial — REAL MOTION", self.phase5)
        self.assertIn("Execute — REAL MOTION", self.phase5)

    def test_phase4_stop_and_execution_evidence_remain_wired(self) -> None:
        for element_id in (
            "digitalTwinPhase4COverall",
            "digitalTwinPhase4CBlockingReasons",
            "digitalTwinPhase5ArtifactStatus",
            "digitalTwinPhase5PreviewInterlock",
            "digitalTwinPhase5GateStatus",
            "digitalTwinPhase5SafeState",
            "digitalTwinPhase5StartMatch",
            "digitalTwinPhase5ExecutionState",
            "digitalTwinPhase5CombinedResult",
            "digitalTwinPhase5BlockingReasons",
            "digitalTwinPhase5ProgressBar",
            "digitalTwinPhase5Diagnostics",
        ):
            self.assertEqual(self.ids.count(element_id), 1)
        stop_tag = re.search(r'<button id="digitalTwinPhase5Stop"[^>]*>', self.phase5)
        self.assertIsNotNone(stop_tag)
        self.assertIn('onclick="stopBoth()"', stop_tag.group(0))
        self.assertIn("STOP BOTH — EXISTING STOP PATH", self.phase5)
        self.assertIn("keep STOP / E-stop ready", self.phase5)
        self.assertIn("DEFERRED AND REMAIN UNVALIDATED — NOT SAFETY CERTIFICATION", self.html)

    def test_feature7_keeps_one_explicit_real_motion_execute_control(self) -> None:
        self.assertEqual(self.ids.count("digitalTwinPhase5Prepare"), 1)
        for removed_id in (
            "digitalTwinPhase5ConfirmationPanel",
            "digitalTwinPhase5ConfirmationTrajectory",
            "digitalTwinPhase5ConfirmationExpiry",
            "digitalTwinPhase5OperatorConfirmed",
            "digitalTwinPhase5Execute",
        ):
            self.assertNotIn(removed_id, self.ids)
        self.assertIn("one explicit final browser", " ".join(self.phase5.split()))

    def test_feature8_cache_version_is_used_without_changing_workflow(self) -> None:
        self.assertIn(
            '/web-assets/digital_twin.js?v=phase6-p6-1-joint-tracking-v1',
            self.html,
        )


if __name__ == "__main__":
    unittest.main()
