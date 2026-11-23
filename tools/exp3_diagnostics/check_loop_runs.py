"""Read-only quality check for runs recorded by the supervised execute loop.
usage: check_loop_runs.py RUN_ID [RUN_ID ...]
Checks per run: identity/binding, execution result, Port10000 continuity, observation
cadence, EXP1/EXP2 coverage and metrics, controller time-base comparison. Then checks
across runs: distinct run/trajectory IDs, same artifact, spacing between cycles."""
import datetime as D
import json, os, sys, statistics as st
sys.path.insert(0, "/home/newtonrattapong/jaka_ws")
from dual_arm_app.backend.experimental_controller_timebase import analyze_controller_timebase
from dual_arm_app.backend.experimental_infrastructure import EXP2_URDF_PATH
BASE = "/home/newtonrattapong/jaka_ws/dual_arm_app/experiment_runs"
def ms(s): return D.datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp() * 1000
rows = []
for run_id in sys.argv[1:]:
    rd = os.path.join(BASE, run_id)
    m = json.load(open(rd + "/manifest.json")); r = json.load(open(rd + "/raw.json"))
    e1 = json.load(open(rd + "/exp1_analysis.json")); e2 = json.load(open(rd + "/exp2_analysis.json"))
    ex = r.get("execution") or {}
    st0 = ex.get("start_time_unix_ns"); dur = ex.get("duration_s")
    print(f"== {run_id} | {m.get('label')} | state={m.get('state')} result={m.get('result',{}).get('status')}")
    print(f"   trajectory_id={ex.get('trajectory_id')} artifact={str(ex.get('artifact_fingerprint'))[:19]} gen={ex.get('artifact_generation')} "
          f"duration={dur}s ARM->exec={(st0/1e6-ms(m['created_at']))/1000:.1f}s")
    print(f"   manifest/raw fingerprints match: {m.get('armed_artifact_fingerprint', m.get('artifact_fingerprint')) in (ex.get('artifact_fingerprint'), None)}"
          f" | planned samples={len(r['planned']['samples'])} | loop meta keys={[k for k in m if 'loop' in k.lower() or 'cycle' in k.lower()]}")
    for side in ("left", "right"):
        P = r["actual_packets"][side]
        t = [p["received_monotonic_ns"] for p in P]; g = [(t[i+1]-t[i])/1e6 for i in range(len(t)-1)]
        seq = [p["packet_sequence_index"] for p in P]
        first = (P[0]["received_at_ms"] - st0/1e6)/1000; last = (P[-1]["received_at_ms"] - st0/1e6)/1000
        print(f"   {side}: packets={len(P)} rate={len(P)/dur:.2f}/s max_gap={max(g):.0f}ms >250={sum(x>250 for x in g)} "
              f"bursts(<5ms)={sum(x<5 for x in g)} seq_breaks={sum(1 for i in range(len(seq)-1) if seq[i+1]-seq[i]!=1)} "
              f"window={first:.2f}..{last:.2f}s")
    obs = r["observations"]; ot = [o["capture_wall_time_ms"] for o in obs]; oi = [ot[i+1]-ot[i] for i in range(len(ot)-1)]
    to = sum(1 for o in obs for s in ("left", "right") if "timeout" in str(((o.get("commanded") or {}).get(s) or {}).get("error") or ""))
    states = [o.get("execution_state") for o in obs]
    print(f"   observations={len(obs)} interval med={st.median(oi):.0f}ms max={max(oi)}ms status_timeouts={to} first/last state={states[0]}/{states[-1]}")
    c = e2["coverage"]; ps = e2["phase_synchronized"]
    print(f"   EXP1 RMSE={e1['combined']['rmse_rad']*57.29578:.3f}deg | EXP2 host usable={c['usable_fraction']*100:.2f}% "
          f"phase usable={ps['coverage']['usable_fraction']*100:.2f}% skipped={c['skipped_status_counts']}")
    tb = analyze_controller_timebase(r, m, EXP2_URDF_PATH)
    meth = tb["methods"]
    fl = [f"{s[0].upper()}#{f['packet_index']}({f['deviation_ms']:+.0f}ms~{f['estimated_stale_cycles_8ms']}x8)" for s in ("left", "right") for f in tb["sides"][s]["flagged_packets"]]
    print(f"   timebase: lag L/R={tb['sides']['left']['tracking_lag_ms']['median']:.1f}/{tb['sides']['right']['tracking_lag_ms']['median']:.1f}ms "
          f"host-ctrl std L/R={tb['sides']['left']['host_minus_controller_time_ms']['std']:.1f}/{tb['sides']['right']['host_minus_controller_time_ms']['std']:.1f}ms flagged={tb['flagged_packet_count']}/{tb['total_packet_count']} {fl}")
    for k in ("host_time", "controller_time", "controller_time_excluding_flagged"):
        x = meth[k]
        print(f"     {k:34s} RMS={x['translation_rms_mm']:.3f} max={x['translation_max_mm']:.2f}mm @{x['translation_max_time_s']:.2f}s "
              f"orient RMS={x['orientation_rms_deg']:.3f} max={x['orientation_max_deg']:.2f}")
    rows.append({"run": run_id, "traj": ex.get("trajectory_id"), "art": ex.get("artifact_fingerprint"), "start": st0, "dur": dur,
                 "plan": json.dumps(r["planned"]["samples"][:3] + r["planned"]["samples"][-3:], sort_keys=True)})
print("== cross-run")
print("   distinct run ids:", len({x['run'] for x in rows}) == len(rows), "| distinct trajectory ids:", len({x['traj'] for x in rows}) == len(rows),
      "| same artifact:", len({x['art'] for x in rows}) == 1, "| same plan (edges):", len({x['plan'] for x in rows}) == 1)
for a, b in zip(rows, rows[1:]):
    print(f"   {a['run'][-8:]} end -> {b['run'][-8:]} start: {(b['start'] - a['start'])/1e9 - a['dur']:.1f}s idle between cycles")
