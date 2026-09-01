"""Feature 4 persistent Center Path Library contracts."""

from pathlib import Path
import json
import subprocess
import unittest
from unittest.mock import patch

from dual_arm_app.backend.center_path_store import (
    CenterPathStore,
    CenterPathStoreError,
    normalize_center_path_name,
    normalize_center_path_payload,
)

ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
STORE_SOURCE = ROOT / "dual_arm_app/backend/center_path_store.py"
VERIFY_STORE = ROOT / ".feature4_verify_store"


def sample_path():
    return {
        "waypoints": [
            {"identifier": "custom", "translation_m": [0.1, 0.2, 0.3], "rpy_rad": [0.0, 0.1, -0.2]},
            {"identifier": "other", "translation_m": [0.2, 0.2, 0.4], "rpy_rad": [0.3, 0.0, 0.1]},
        ],
        "fixed_orientation_rpy_rad": [0.0, 0.0, 0.0],
        "orientation_source": "TEST",
        "segment_duration_s": 2.5,
        "samples_per_segment": 4,
        "candidate_attempts_per_arm": 3,
    }


class CenterPathStoreTests(unittest.TestCase):
    def test_name_validation_blocks_traversal_but_allows_unicode(self):
        self.assertEqual(normalize_center_path_name("  เส้นทาง A  "), "เส้นทาง A")
        for bad in ("", "../escape", "a/b", "a\\b", ".."):
            with self.subTest(bad=bad), self.assertRaises(CenterPathStoreError):
                normalize_center_path_name(bad)

    def test_payload_is_validated_and_waypoints_are_auto_renumbered(self):
        normalized = normalize_center_path_payload(sample_path())
        self.assertEqual([item["identifier"] for item in normalized["waypoints"]], ["W0", "W1"])
        self.assertEqual(normalized["segment_duration_s"], 2.5)
        with self.assertRaises(CenterPathStoreError):
            normalize_center_path_payload({"waypoints": []})

    def test_workspace_store_save_load_list_and_mocked_delete(self):
        store = CenterPathStore(VERIFY_STORE)
        saved = store.save("Feature4_ทดสอบ", sample_path(), overwrite=True)
        self.assertTrue(saved["ok"])
        loaded = store.load("Feature4_ทดสอบ")
        self.assertEqual(loaded["document"]["path"]["waypoints"][1]["identifier"], "W1")
        listed = store.list()
        self.assertTrue(any(item["name"] == "Feature4_ทดสอบ" for item in listed["paths"]))
        with self.assertRaises(CenterPathStoreError):
            store.save("Feature4_ทดสอบ", sample_path(), overwrite=False)
        with patch.object(Path, "unlink") as mocked_unlink:
            deleted = store.delete("Feature4_ทดสอบ")
            self.assertTrue(deleted["ok"])
            mocked_unlink.assert_called_once()

    def test_delete_is_scoped_to_saved_path_file(self):
        source = STORE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("path.unlink()", source)
        self.assertNotIn("shutil.rmtree", source)
        self.assertNotIn("os.remove", source)
        self.assertNotIn("subprocess", source)


class CenterPathApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.backend = BACKEND.read_text(encoding="utf-8")
        cls.html = (WEB / "index.html").read_text(encoding="utf-8")
        cls.controller = (WEB / "digital_twin.js").read_text(encoding="utf-8")

    def test_backend_routes_are_explicitly_no_motion(self):
        for route in (
            "/api/digital-twin/center-paths",
            "/api/digital-twin/center-path/save",
            "/api/digital-twin/center-path/load",
            "/api/digital-twin/center-path/rename",
            "/api/digital-twin/center-path/delete",
        ):
            self.assertIn(route, self.backend)
        self.assertIn('"/api/digital-twin/center-path"', self.backend)

    def test_step2_has_persistent_library_controls_and_new_cache_version(self):
        for element_id in (
            "digitalTwinCenterPathSelect", "digitalTwinCenterPathName",
            "digitalTwinCenterPathSave", "digitalTwinCenterPathSaveAs",
            "digitalTwinCenterPathLoad", "digitalTwinCenterPathRename",
            "digitalTwinCenterPathDelete", "digitalTwinCenterPathStatus",
        ):
            self.assertIn(f'id="{element_id}"', self.html)
        self.assertRegex(
            self.html,
            r'/web-assets/digital_twin\.js\?v=[A-Za-z0-9._-]+',
        )
        self.assertIn("digital_twin_center_path_library.js", self.controller)
        self.assertIn("survives browser refresh", self.html)
        self.assertIn("Path storage is NO MOTION", self.html)

    def test_controller_restores_path_and_invalidates_stale_plan(self):
        for token in (
            "loadSelectedCenterPath", "setObjectPlanningWaypoints(stored.waypoints",
            "writeObjectPlanningConfigurationControls", "centerPathLibraryState.dirty",
            "window.confirm", "saveAs: true", "saveAs: false",
        ):
            self.assertIn(token, self.controller)

    def test_browser_api_module_uses_only_path_library_routes(self):
        module = (WEB / "digital_twin_center_path_library.js").read_text(encoding="utf-8")
        self.assertNotIn("execute", module.lower())
        self.assertNotIn("joint_move", module)
        self.assertNotIn("servo_j", module)
        script = r'''
import {
  listCenterPaths, saveCenterPath, loadCenterPath, renameCenterPath, deleteCenterPath,
} from "./dual_arm_app/web/digital_twin_center_path_library.js";
const calls = [];
const fakeFetch = async (url, options) => {
  calls.push({url, method: options.method, body: options.body ? JSON.parse(options.body) : null});
  return {ok: true, status: 200, json: async () => ({ok: true, paths: [], name: "P"})};
};
await listCenterPaths(fakeFetch);
await saveCenterPath("P", {waypoints: []}, true, fakeFetch);
await loadCenterPath("P", fakeFetch);
await renameCenterPath("P", "Q", fakeFetch);
await deleteCenterPath("Q", fakeFetch);
console.log(JSON.stringify(calls));
'''
        result = subprocess.run(
            ["node", "--input-type=module", "-e", script],
            cwd=ROOT, capture_output=True, text=True, check=True,
        )
        calls = json.loads(result.stdout.strip())
        self.assertEqual([item["method"] for item in calls], ["GET", "POST", "POST", "POST", "POST"])
        self.assertTrue(all("center-path" in item["url"] for item in calls))
        self.assertEqual(calls[1]["body"]["overwrite"], True)
        self.assertEqual(calls[3]["body"]["new_name"], "Q")


if __name__ == "__main__":
    unittest.main()
