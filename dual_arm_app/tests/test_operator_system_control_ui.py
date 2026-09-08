from __future__ import annotations

import re
import subprocess
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HTML_PATH = ROOT / "dual_arm_app/web/index.html"
JS_PATH = ROOT / "dual_arm_app/web/operator_system_control.js"
HTML = HTML_PATH.read_text(encoding="utf-8")
JS = JS_PATH.read_text(encoding="utf-8")
BASE = subprocess.check_output(
    ["git", "show", "HEAD:dual_arm_app/web/index.html"], cwd=ROOT, text=True
)


def ids(source: str) -> list[str]:
    return re.findall(r'id="([^"]+)"', source)


def inline_handlers(source: str) -> list[str]:
    return re.findall(
        r'\b(?:onclick|onchange|oninput|onmousedown|onmouseup|onpointerdown|onpointerup)="[^"]*"',
        source,
    )
def test_all_existing_ids_and_inline_handlers_are_preserved():
    base_ids = set(ids(BASE))
    current_ids = ids(HTML)
    replaced_system_action_ids = {
        "systemControlLeftPowerOn", "systemControlLeftPowerOff",
        "systemControlLeftEnable", "systemControlLeftDisable",
        "systemControlRightPowerOn", "systemControlRightPowerOff",
        "systemControlRightEnable", "systemControlRightDisable",
    }
    assert (base_ids - replaced_system_action_ids).issubset(current_ids)
    assert not (replaced_system_action_ids & set(current_ids))
    assert inline_handlers(BASE) == inline_handlers(HTML)
    counts = Counter(current_ids)
    assert not [name for name, count in counts.items() if count != 1]

def test_existing_script_sources_preserved_and_system_control_added_once():
    pattern = r'<script[^>]+src="([^"]+)"'
    before = re.findall(pattern, BASE)
    after = re.findall(pattern, HTML)
    assert after == before
    assert after.count("/web-assets/operator_system_control.js?v=split-system-control-v1") == 1

def test_split_workspace_is_twin_left_55_operator_right_45_and_keyboard_accessible():
    assert '--operator-share: 45%' in HTML
    assert '--twin-share: 55%' in HTML
    split = HTML.index('id="digitalTwinSplitWorkspace"')
    viewer = HTML.index('id="digitalTwinViewerPane"', split)
    divider_pos = HTML.index('id="digitalTwinSplitDivider"', split)
    operator = HTML.index('id="digitalTwinOperatorPane"', split)
    assert split < viewer < divider_pos < operator
    divider = HTML.split('id="digitalTwinSplitDivider"', 1)[1].split("></div>", 1)[0]
    assert 'role="separator"' in divider
    assert 'aria-orientation="vertical"' in divider
    assert 'aria-valuenow="55"' in divider
    assert 'Digital Twin 55 percent; Operator workspace 45 percent' in divider
    assert 'tabindex="0"' in divider
    assert 'ArrowLeft' in JS and 'ArrowRight' in JS
    assert 'digitalTwinSplitReset' in JS
    assert 'SPLIT_VIEWER_MIN_PX = 600' in JS
    assert 'SPLIT_OPERATOR_MIN_PX = 520' in JS
def test_existing_viewer_moved_once_and_experiments_remain_outside_split():
    assert ids(HTML).count("digitalTwinViewer") == 1
    assert ids(HTML).count("digitalTwinStatus") == 1
    split_start = HTML.index('id="digitalTwinSplitWorkspace"')
    experiment = HTML.index('id="digitalTwinExperimentPanel"')
    dialog = HTML.index('id="systemControlConfirmDialog"')
    assert split_start < dialog < experiment
    assert HTML.index('id="digitalTwinViewer"', split_start) < dialog


def test_system_control_required_status_and_action_ids_exist():
    required = {
        "systemControlConnectRobots", "systemControlLeftConnection", "systemControlLeftPower",
        "systemControlLeftEnableState", "systemControlLeftMotion", "systemControlLeftFeedback",
        "systemControlLeftFeedbackAge", "systemControlRightConnection", "systemControlRightPower",
        "systemControlRightEnableState", "systemControlRightMotion", "systemControlRightFeedback",
        "systemControlRightFeedbackAge", "systemControlMoveItState", "systemControlMoveItSource",
        "systemControlWebState", "systemControlWebSource", "systemControlShutdownState",
        "systemControlSafeToClose", "systemControlShutdownReason", "systemControlResetMoveIt",
        "systemControlConfirmDialog", "systemControlActionDetail",
    }
    current = set(ids(HTML))
    assert required <= current
    for side in ("Left", "Right"):
        assert f"systemControl{side}PowerToggle" in current
        assert f"systemControl{side}EnableToggle" in current
def test_system_control_js_uses_only_allowlisted_system_control_endpoints():
    endpoints = set(re.findall(r'[`"](/api/system-control[^`"$]*)', JS))
    assert endpoints == {
        "/api/system-control/status",
        "/api/system-control/connect-robots",
        "/api/system-control/reset-moveit",
        "/api/system-control/robot/",
    }
    forbidden = ("/api/stop", "/api/home", "/api/jog", "/api/direct", "/phase5/execute", "Port10000")
    assert all(token not in JS for token in forbidden)
    assert 'typeof enabled !== "boolean"' in JS
    assert "{ enabled }" in JS
    assert "Boolean(enabled)" not in JS


def test_page_initialization_is_read_only_and_mutations_are_click_bound():
    initialization = JS.rsplit("ui.resetSplit?.addEventListener", 1)[1]
    tail = initialization.rsplit("readStatus();", 1)[1]
    assert "setInterval(pollStatus" in tail
    assert "postJson(" not in tail
    assert 'method: "GET"' in JS
    assert 'method: "POST"' in JS
    assert 'systemControlConnectRobots" class="system-control-connect" type="button" disabled' in HTML


def test_risky_actions_use_app_owned_dialog_not_browser_confirm():
    assert '<dialog\n    id="systemControlConfirmDialog"' in HTML
    assert "showModal()" in JS
    assert not re.search(r"(?:window\.)?(?:alert|confirm|prompt)\s*\(", JS)
    assert "Power OFF" in JS and "Disable" in JS and "Reset MoveIt" in JS


def test_status_polling_is_non_overlapping_fail_closed_and_snapshot_expires():
    assert "if (statusInFlight && !force) return null" in JS
    assert "STATUS_SNAPSHOT_MAX_AGE_MS" in JS
    assert "System status snapshot expired" in JS
    poll = JS.split("function pollStatus()", 1)[1].split("readStatus();", 1)[0]
    assert "if (latestStatus && latestStatusReceivedAt > 0" in poll
    assert "if (!mutationBusy && latestStatus" not in poll
    assert "do not repeat the action until fresh status returns" in JS
    assert "const refreshed = await readStatus({ announceFailure: true, force: true })" in JS
    assert 'actual?.received_at_ms != null ? " · STALE"' in JS
    verified = JS.split("function verifiedArmActual", 1)[1].split("function openConfirmation", 1)[0]
    assert "STATUS_SNAPSHOT_MAX_AGE_MS" in verified
    assert 'driver?.connection === "CONNECTED"' in verified
    assert "actual?.fresh === true" in verified
    assert "latestStatus?.shutdown?.safe === true" in verified


def test_pending_confirmation_cannot_escape_and_split_uses_dynamic_bounds():
    cancel = JS.split('ui.dialog?.addEventListener("cancel"', 1)[1].split("});", 1)[0]
    assert "if (!mutationBusy) closeConfirmation()" in cancel
    assert "splitBounds()" in JS
    assert "dynamicMin" in JS and "dynamicMax" in JS
    assert '(min-width: 1321px)' in JS
    assert '@media (max-width: 1320px)' in HTML
    assert 'height: min(720px, calc(100dvh - 244px))' in HTML



def test_system_control_is_compact_header_content_not_workflow_content():
    header_start = HTML.index('<header class="hmi-command-header">')
    header_end = HTML.index('</header>', header_start)
    panel = HTML.index('id="systemControlPanel"')
    operator = HTML.index('id="digitalTwinOperatorPane"')
    assert header_start < panel < header_end < operator
    header_block = HTML[header_start:header_end]
    assert "SYSTEM CONTROL" in header_block
    assert "CONNECT ROBOTS" in header_block
    assert "RESET MoveIt" in header_block
    assert "Actual state only" not in header_block
    assert "Runtime ownership" not in header_block


def test_system_control_uses_one_power_and_one_enable_toggle_per_arm():
    current = ids(HTML)
    for side in ("Left", "Right"):
        assert current.count(f"systemControl{side}PowerToggle") == 1
        assert current.count(f"systemControl{side}EnableToggle") == 1
        assert f"systemControl{side}PowerOn" not in current
        assert f"systemControl{side}PowerOff" not in current
        assert f"systemControl{side}Enable" not in current
        assert f"systemControl{side}Disable" not in current
    assert 'aria-pressed' in JS
    assert 'POWER: UNKNOWN' in JS and 'ENABLE: UNKNOWN' in JS
    assert 'actual.power === false' in JS
    assert 'actual.enabled === false' in JS
    assert 'openConfirmation(' in JS


def test_manual_jog_moved_out_of_technical_workspace_into_side_rail():
    tech_start = HTML.index('id="hmiTechnicalWorkspace"')
    tech_end = HTML.index('</details>', tech_start)
    jog_start = HTML.index('id="hmiJogSidebar"')
    split_start = HTML.index('id="digitalTwinSplitWorkspace"')
    assert tech_end < jog_start < split_start
    technical = HTML[tech_start:tech_end]
    jog_end = HTML.index('</details>', jog_start)
    jog = HTML[jog_start:jog_end]
    assert "Dual JAKA A12 Manual Jog" not in technical
    assert "Dual JAKA A12 Manual Jog" in jog
    for jog_id in (
        "side-left", "side-right", "side-both", "coord-joint", "coord-base", "coord-tool",
        "syncMode", "jogSpeedHeading", "linearSpeedPercent", "linearSpeedPercentValue",
        "rotateSpeedPercent", "rotateSpeedPercentValue", "jointSpeedPercent",
        "jointSpeedPercentValue", "moveSpeedHeading", "moveVelPercent",
        "moveVelPercentValue", "moveAccPercent", "moveAccPercentValue", "linearSpeed",
        "rotateSpeed", "jointSpeed", "moveVel", "moveAcc", "jointButtons", "tcpButtons",
    ):
        assert ids(HTML).count(jog_id) == 1
        assert f'id="{jog_id}"' in jog
    assert '.hmi-jog-sidebar {' in HTML
    assert 'position: fixed' in HTML.split('.hmi-jog-sidebar {', 1)[1].split('}', 1)[0]


def test_top_level_surfaces_are_fluid_without_1800px_island_cap():
    assert 'min(1800px' not in HTML
    fluid_rule = HTML.split('body > .hmi-technical-workspace,', 1)[1].split('}', 1)[0]
    assert 'width: calc(100% - clamp(' in fluid_rule
    assert 'max-width: 1800px' not in fluid_rule
    assert 'zoom:' not in HTML
    assert 'transform: scale(' not in HTML
