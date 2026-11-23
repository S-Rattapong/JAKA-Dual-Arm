"""Offline reproduction: production Port10000ActualFeedback receiver + production
ExperimentRunStore.list_runs() in ONE Python process (as in the backend), fed by a
separate fake-controller process on 127.0.0.1. Read-only on experiment data.

usage: python3 -B harness.py WS_ROOT RUNS_DIR OUT_JSON MODE
MODE: baseline | list_runs | manifest_only
"""
import json, math, os, subprocess, sys, threading, time
from pathlib import Path

ws, runs_dir, out, mode = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
sys.path.insert(0, ws)
from dual_arm_app.backend.port10000_actual_feedback import Port10000ActualFeedback  # production
from dual_arm_app.backend.experimental_infrastructure import ExperimentRunStore    # production

HERE = Path(__file__).parent
PL, PR, PERIOD_MS = 47101, 47102, 100.0
PRE_S, POST_S = 6.0, 6.0

recv = {"left": [], "right": []}
def observer(p):
    idx = round((math.degrees(p["actual_joints_rad"][0]) - 10.0) / 0.001)
    recv[p["side"]].append((p["received_monotonic_ns"], idx))

sendlog = str(HERE / f"send_{mode}.txt")
total_s = PRE_S + 60 + POST_S
srv = subprocess.Popen([sys.executable, "-B", str(HERE / "fake_controller.py"), str(PL), str(PR), str(PERIOD_MS), str(total_s), sendlog])
time.sleep(0.5)
errors = []
fb = Port10000ActualFeedback({"left": ("127.0.0.1", PL), "right": ("127.0.0.1", PR)},
                             connect_timeout_s=0.5, recv_timeout_s=0.5, reconnect_backoff_s=1.0,
                             stale_reconnect_timeout_s=2.0, log_warning=errors.append)
fb.subscribe_packets(observer)
fb.start()
time.sleep(PRE_S)

phase = {"mode": mode}
t0 = time.monotonic_ns()
if mode == "list_runs":
    rows = ExperimentRunStore(runs_dir).list_runs()
    phase["rows"] = len(rows)
elif mode == "list_runs_loop":
    store = ExperimentRunStore(runs_dir); n = 0
    while time.monotonic_ns() - t0 < 15e9:
        rows = store.list_runs(); n += 1
    phase["rows"] = len(rows); phase["calls"] = n
elif mode == "manifest_only":
    rows = []
    for d in sorted(Path(runs_dir).iterdir(), reverse=True):
        m = d / "manifest.json"
        if m.is_file():
            man = json.loads(m.read_text(encoding="utf-8"))
            rows.append({"run_id": d.name, "label": man.get("label"), "state": man.get("state")})
    phase["rows"] = len(rows)
else:
    time.sleep(20.0)
t1 = time.monotonic_ns()
phase["work_start_ns"], phase["work_end_ns"] = t0, t1
phase["work_s"] = (t1 - t0) / 1e9
time.sleep(POST_S)
fb.stop()
srv.terminate(); srv.wait(timeout=5)
json.dump({"phase": phase, "recv": recv, "errors": errors[:50]}, open(out, "w"))
print(json.dumps({"mode": mode, "work_s": round(phase["work_s"], 2), "rows": phase.get("rows"),
                  "n_left": len(recv["left"]), "n_right": len(recv["right"]), "errors": len(errors)}))
