from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BACKEND = (ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py").read_text(encoding="utf-8")


class BackendLifecycleContractTests(unittest.TestCase):
    def test_backend_ros_node_name_is_single_canonical_name(self):
        self.assertIn('super().__init__("dual_jaka_web_backend")', BACKEND)
        self.assertEqual(BACKEND.count("node = DualJakaWebNode(load_config())"), 1)

    def test_single_instance_guard_precedes_ros_node_creation(self):
        lock = BACKEND.index("_BACKEND_INSTANCE_LOCK_PATH")
        guard = BACKEND.index("fcntl.LOCK_EX | fcntl.LOCK_NB")
        node = BACKEND.index("node = DualJakaWebNode(load_config())")
        self.assertLess(lock, guard)
        self.assertLess(guard, node)
        self.assertIn(".dual_jaka_web_backend.instance.lock", BACKEND)
        self.assertIn("Another dual_jaka_web_backend process already owns", BACKEND)

    def test_guard_is_workspace_local_and_does_not_delete_lock_file(self):
        bootstrap = BACKEND[BACKEND.index("_BACKEND_INSTANCE_LOCK_PATH"):BACKEND.index("node = DualJakaWebNode(load_config())")]
        self.assertIn("Path(__file__).resolve().parent", bootstrap)
        self.assertNotIn("unlink(", bootstrap)
        self.assertNotIn("remove(", bootstrap)


if __name__ == "__main__":
    unittest.main()
