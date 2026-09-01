"""Feature 5 regression tests for the unified Live Mirror operator switch."""

from __future__ import annotations

import base64
import json
import subprocess
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
APP_ROOT = REPOSITORY_ROOT / "dual_arm_app"
LIVE_SOURCE_PATH = APP_ROOT / "web/digital_twin_live_source.js"
HTML_PATH = APP_ROOT / "web/index.html"


class Feature5LiveMirrorSwitchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = LIVE_SOURCE_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_quick_switch_is_outside_diagnostics_and_advanced_controls_remain(self) -> None:
        switch_id = 'id="digitalTwinLiveMirrorSwitch"'
        self.assertEqual(self.html.count(switch_id), 1)
        switch_position = self.html.index(switch_id)
        diagnostics_position = self.html.index(
            '<details class="digital-twin-operator-details"'
        )
        self.assertLess(switch_position, diagnostics_position)
        for label in (
            "Live Mirror",
            "ON — LIVE FEEDBACK + MIRROR",
            "Start Live Feedback — READ ONLY",
            "Stop Live Feedback",
            "Enable Mirror",
            "Disable Mirror",
        ):
            with self.subTest(label=label):
                self.assertIn(label, self.html + self.source)
        self.assertIn("feature5-live-mirror-switch-v1", self.html)

    def test_unified_switch_source_is_read_only(self) -> None:
        self.assertIn("function setUnifiedLiveMirrorEnabled(enabled)", self.source)
        self.assertIn('digitalTwin.setMirrorEnabled(false)', self.source)
        self.assertIn('digitalTwin.setMirrorEnabled(true)', self.source)
        self.assertNotIn('method: "POST"', self.source)
        for route in (
            "/api/jog",
            "/api/home",
            "/api/stop",
            "/api/direct",
            "/api/waypoint",
            "/api/program",
            "/api/digital-twin/phase5/execute",
        ):
            with self.subTest(route=route):
                self.assertNotIn(route, self.source)

    def test_combined_switch_runtime_contract(self) -> None:
        module_url = "data:text/javascript;base64," + base64.b64encode(
            self.source.encode("utf-8")
        ).decode("ascii")
        harness = f'''
const elements = new Map();
function getElement(id) {{
  if (!elements.has(id)) {{
    elements.set(id, {{
      id,
      textContent: "",
      checked: false,
      indeterminate: false,
      attributes: {{}},
      dataset: {{}},
      listeners: {{}},
      addEventListener(name, callback) {{ this.listeners[name] = callback; }},
      setAttribute(name, value) {{ this.attributes[name] = String(value); }},
    }});
  }}
  return elements.get(id);
}}
globalThis.document = {{ getElementById: getElement }};
let mirrorEnabled = false;
const mirrorCalls = [];
let ingested = 0;
let nextTimerId = 1;
const timers = new Map();
globalThis.window = {{
  setTimeout(callback, delay) {{
    const id = nextTimerId++;
    timers.set(id, {{ callback, delay }});
    return id;
  }},
  clearTimeout(id) {{ timers.delete(id); }},
  dualArmDigitalTwin: {{
    getMirrorState() {{ return {{ enabled: mirrorEnabled }}; }},
    setMirrorEnabled(value) {{
      mirrorEnabled = value === true;
      mirrorCalls.push(mirrorEnabled);
      return {{ enabled: mirrorEnabled }};
    }},
    ingestStatusSnapshot() {{ ingested += 1; }},
  }},
}};
globalThis.fetch = async () => ({{
  ok: true,
  status: 200,
  json: async () => ({{
    ok: true,
    source: "ros_joint_state_cache",
    left: {{ valid: true, joint: [1,2,3,4,5,6], received_at_ms: 900 }},
    right: {{ valid: true, joint: [-1,-2,-3,-4,-5,-6], received_at_ms: 950 }},
  }}),
}});
const live = await import({json.dumps(module_url)});
const toggle = getElement("digitalTwinLiveMirrorSwitch");
const status = getElement("digitalTwinLiveMirrorQuickStatus");
const panel = getElement("digitalTwinLiveMirrorQuick");
const initial = {{
  checked: toggle.checked,
  indeterminate: toggle.indeterminate,
  status: status.textContent,
  panelState: panel.dataset.state,
}};
live.setUnifiedLiveMirrorEnabled(true);
await Promise.resolve();
await Promise.resolve();
const enabled = {{
  running: live.getLiveFeedbackState().running,
  mirrorEnabled,
  checked: toggle.checked,
  indeterminate: toggle.indeterminate,
  status: status.textContent,
  panelState: panel.dataset.state,
}};
live.stopLiveFeedback();
const partial = {{
  running: live.getLiveFeedbackState().running,
  mirrorEnabled,
  checked: toggle.checked,
  indeterminate: toggle.indeterminate,
  ariaChecked: toggle.attributes["aria-checked"],
  status: status.textContent,
}};
live.setUnifiedLiveMirrorEnabled(false);
const disabled = {{
  running: live.getLiveFeedbackState().running,
  mirrorEnabled,
  checked: toggle.checked,
  indeterminate: toggle.indeterminate,
  status: status.textContent,
  panelState: panel.dataset.state,
}};
console.log(JSON.stringify({{
  initial,
  enabled,
  partial,
  disabled,
  mirrorCalls,
  ingested,
}}));
'''
        result = subprocess.run(
            ["node", "--input-type=module", "-e", harness],
            cwd=REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        output = json.loads(result.stdout)
        self.assertEqual(
            output["initial"],
            {
                "checked": False,
                "indeterminate": False,
                "status": "OFF — FEEDBACK + MIRROR STOPPED",
                "panelState": "off",
            },
        )
        self.assertTrue(output["enabled"]["running"])
        self.assertTrue(output["enabled"]["mirrorEnabled"])
        self.assertTrue(output["enabled"]["checked"])
        self.assertFalse(output["enabled"]["indeterminate"])
        self.assertEqual(output["enabled"]["status"], "ON — LIVE FEEDBACK + MIRROR")
        self.assertEqual(output["enabled"]["panelState"], "on")

        self.assertFalse(output["partial"]["running"])
        self.assertTrue(output["partial"]["mirrorEnabled"])
        self.assertFalse(output["partial"]["checked"])
        self.assertTrue(output["partial"]["indeterminate"])
        self.assertEqual(output["partial"]["ariaChecked"], "mixed")
        self.assertEqual(
            output["partial"]["status"],
            "PARTIAL — FEEDBACK OFF / MIRROR ON",
        )

        self.assertFalse(output["disabled"]["running"])
        self.assertFalse(output["disabled"]["mirrorEnabled"])
        self.assertFalse(output["disabled"]["checked"])
        self.assertFalse(output["disabled"]["indeterminate"])
        self.assertEqual(output["disabled"]["status"], "OFF — FEEDBACK + MIRROR STOPPED")
        self.assertEqual(output["disabled"]["panelState"], "off")
        self.assertEqual(output["mirrorCalls"], [True, False])


if __name__ == "__main__":
    unittest.main()
