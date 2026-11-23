"""Offline: does recorder persistence (ARM create / CLOSE update) stall Port10000 receive?

Same set-up as harness.py (fake 10 Hz pusher in a separate process + production
Port10000ActualFeedback in this process), but the work is the production
ExperimentRunStore persistence of a real run's payload, written to a TEMPORARY
directory (never to experiment_runs).
usage: harness_persist.py WS SOURCE_RUN_DIR OUT_JSON MODE   MODE: create | close | load_response
"""
import copy, json, math, shutil, subprocess, sys, tempfile, time
from pathlib import Path

ws, src_dir, out, mode = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
sys.path.insert(0, ws)
from dual_arm_app.backend.port10000_actual_feedback import Port10000ActualFeedback
from dual_arm_app.backend import experimental_infrastructure as ei

HERE = Path(__file__).parent
PL, PR = 47111, 47112
recv = {"left": [], "right": []}
def observer(p):
    recv[p["side"]].append(p["received_monotonic_ns"])

srv = subprocess.Popen([sys.executable, "-B", str(HERE / "fake_controller.py"), str(PL), str(PR), "100", "40",
                        str(Path(out).with_suffix(".send.txt"))])
time.sleep(0.5)
fb = Port10000ActualFeedback({"left": ("127.0.0.1", PL), "right": ("127.0.0.1", PR)}, reconnect_backoff_s=1.0)
fb.subscribe_packets(observer); fb.start()
manifest = json.load(open(Path(src_dir) / "manifest.json")); raw = json.load(open(Path(src_dir) / "raw.json"))
tmp = Path(tempfile.mkdtemp(prefix="persist_", dir="/tmp/claude_audit"))
store = ei.ExperimentRunStore(tmp)
time.sleep(4.0)
marks = []
def mark(name, fn):
    t0 = time.monotonic_ns(); fn(); t1 = time.monotonic_ns(); marks.append((name, t0, t1))
run_id = manifest["run_id"]
if mode == "create":
    m2 = copy.deepcopy(manifest); r2 = copy.deepcopy(raw)
    mark("deepcopy(raw)", lambda: copy.deepcopy(raw))
    mark("store.create", lambda: store.create(m2, r2))
elif mode == "load_response":
    # What GET /api/experiments/{run_id} does: load every file, then FastAPI encodes the response.
    from fastapi.encoders import jsonable_encoder
    real = ei.ExperimentRunStore(Path(src_dir).parent)
    holder = {}
    mark("store.load", lambda: holder.update(run=real.load(manifest["run_id"])))
    mark("jsonable_encoder", lambda: holder.update(enc=jsonable_encoder({"ok": True, "run": holder["run"]})))
    mark("json.dumps(response)", lambda: holder.update(body=json.dumps(holder["enc"], ensure_ascii=False, allow_nan=False, separators=(",", ":"))))
    print("response MB", len(holder["body"]) / 1e6)
else:
    store.create(manifest, raw); time.sleep(2.0)
    mark("store.update(raw)", lambda: store.update(run_id, manifest, raw))
    aligned = {}
    mark("align_planned_to_actual", lambda: aligned.update(ei.align_planned_to_actual(raw)))
    mark("store.update(raw,aligned)", lambda: store.update(run_id, manifest, raw, aligned))
    mark("analyze_rigid_grasp", lambda: ei.analyze_rigid_grasp(raw, manifest, ei.EXP2_URDF_PATH))
time.sleep(3.0)
fb.stop(); srv.terminate(); srv.wait(timeout=5)
shutil.rmtree(tmp, ignore_errors=True)
res = {"marks": [(n, (b - a) / 1e6) for n, a, b in marks]}
for side in recv:
    t = recv[side]
    for n, a, b in marks:
        inside = [x for x in t if a - 2e8 <= x <= b + 6e8]
        gaps = [(inside[i + 1] - inside[i]) / 1e6 for i in range(len(inside) - 1)]
        res[f"{side}:{n}"] = {"max_gap_ms": max(gaps) if gaps else None}
json.dump(res, open(out, "w"))
print(json.dumps(res))
