"""Exercise the supervised worker and on-disk recorder with an offline coordinator."""
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dual_arm_app.tests.test_phase5_supervised_loop import make_manager, await_state
from dual_arm_app.tests.test_exp0_experimental_infrastructure import artifact, report
from dual_arm_app.backend.experimental_infrastructure import ExperimentRecorder, ExperimentRunStore


class LoopRecordingTests(unittest.TestCase):
    def test_30_separate_completed_recordings(self):
        with TemporaryDirectory() as folder:
            recorder = ExperimentRecorder(ExperimentRunStore(Path(folder)))
            manager, coordinator, _, aborts = make_manager(complete_after=1, settle_s=0, heartbeat_timeout_s=60)
            ids = []

            def arm(metadata):
                state = recorder.arm(artifact=artifact(), artifact_generation=1, phase4_report=report(),
                                     phase4_generation=1, metadata=metadata)
                ids.append(state["run_id"])
                return state

            def execute(confirmed):
                response = coordinator.execute(confirmed)
                recorder.bind_execution({**response["execution"], "artifact_generation": 1,
                                         "artifact_fingerprint": artifact().artifact_fingerprint})
                for side in ("left", "right"):
                    recorder.ingest_actual_packet({"side": side, "joints_rad": [0] * 6, "received_at_ms": len(ids)})
                return response

            def finish(run_id, tid, result):
                result["driver_feedback"]["terminal"] = True
                return recorder.finish_loop_cycle(run_id, tid, result, {}, {})

            manager.arm_recording = arm
            manager.execute_callback = execute
            manager.finish_recording = finish
            manager.disarm_recording = recorder.disarm
            session = manager.start(mode="count", count=30, operator_confirmed=True, recording={"label": "NOM"})
            status = await_state(manager, {"COMPLETED", "FAILED"}, 60)
            self.assertEqual(status.get("saved_cycles"), 30, status)
            self.assertEqual(len(set(ids)), 30)
            for index, run_id in enumerate(ids, 1):
                run = recorder.store.load(run_id)
                manifest = run["manifest"]
                self.assertEqual(manifest["loop_cycle"], {"session_id": session["id"], "index": index, "count": 30})
                self.assertEqual(manifest["label"], f"NOM Run{index:02d}")
                self.assertEqual(manifest["result"]["trajectory_id"], f"test-trajectory-{index}")
                for side in ("left", "right"):
                    packets = run["raw"]["actual_packets"][side]
                    self.assertEqual([p["received_at_ms"] for p in packets], [index])
            self.assertFalse(aborts)

    def test_failed_save_blocks_next_cycle(self):
        manager, coordinator, _, aborts = make_manager(complete_after=1)
        manager.arm_recording = lambda metadata: {"run_id": "test"}
        def failed_save(*args):
            raise OSError("disk full")
        manager.finish_recording = failed_save
        manager.disarm_recording = lambda run_id: None
        manager.start(mode="count", count=30, operator_confirmed=True, recording={})
        self.assertIn("disk full", await_state(manager, {"FAILED"})["reason"])
        self.assertEqual(coordinator.execute_count, 1)
        self.assertFalse(aborts)
