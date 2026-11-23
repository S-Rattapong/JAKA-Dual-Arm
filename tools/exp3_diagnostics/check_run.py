"""Read-only quality check of one saved run (by label) vs the earlier metrics."""
import json, os, sys, statistics as st, datetime as D
base = "/home/newtonrattapong/jaka_ws/dual_arm_app/experiment_runs"
label = sys.argv[1]
cands = []
for d in sorted(os.listdir(base)):
    p = os.path.join(base, d, "manifest.json")
    if os.path.isfile(p):
        m = json.load(open(p))
        if (m.get("label") or "").strip().lower() == label.lower():
            cands.append((d, m))
print("matches:", [c[0] for c in cands])
d, m = cands[-1]
rd = os.path.join(base, d)
print("run", d, "| state", m.get("state"), "| result", (m.get("result") or {}).get("status"), "| files", sorted(os.listdir(rd)))
r = json.load(open(os.path.join(rd, "raw.json")))
ex = r.get("execution") or {}
arm = D.datetime.fromisoformat(m["created_at"].replace("Z", "+00:00")).timestamp() * 1000
st0 = ex.get("start_time_unix_ns"); dur = ex.get("duration_s")
print(f"duration {dur}s | ARM->execute {(st0/1e6-arm)/1000:.1f}s | endpoint_evidence: {'endpoint_evidence' in r}")
gs = {}
for side in ("left", "right"):
    P = r["actual_packets"][side]
    t = [p["received_monotonic_ns"] for p in P]
    g = [(t[i+1]-t[i])/1e6 for i in range(len(t)-1)]
    seq = [p["packet_sequence_index"] for p in P]
    gs[side] = [P[i]["received_at_ms"] for i in range(len(g)) if g[i] > 250]
    print(f"{side}: packets={len(P)} rate={len(P)/dur:.2f}/s max_gap={max(g):.1f}ms p50={st.median(g):.1f}ms "
          f">250={sum(x>250 for x in g)} >150={sum(x>150 for x in g)} <5ms={sum(x<5 for x in g)} "
          f"seq_breaks={sum(1 for i in range(len(seq)-1) if seq[i+1]-seq[i]!=1)} monotonic={all(x>=0 for x in g)}")
obs = r["observations"]
ot = [o["capture_wall_time_ms"] for o in obs]
oi = [ot[i+1]-ot[i] for i in range(len(ot)-1)]
to = sum(1 for o in obs for s in ("left","right") if "timeout" in str(((o.get("commanded") or {}).get(s) or {}).get("error") or ""))
print(f"observations={len(obs)} interval med={st.median(oi):.0f}ms max={max(oi)}ms status-timeouts={to} states={sorted(set(o.get('execution_state') for o in obs))}")
for f in ("exp1_analysis.json", "exp2_analysis.json"):
    p = os.path.join(rd, f)
    if not os.path.isfile(p): print(f, "MISSING"); continue
    a = json.load(open(p))
    if f.startswith("exp1"):
        print(f"EXP1 combined RMSE={a['combined']['rmse_rad']*57.29578:.3f} deg coverage={json.dumps(a.get('coverage'))[:200]}")
    else:
        c = a["coverage"]; ps = a["phase_synchronized"]
        print(f"EXP2 host usable={c['usable_fraction']*100:.2f}% skipped={c['skipped_status_counts']} max_source_gap={c['max_source_gap_ms']}")
        print(f"     phase usable={ps['coverage']['usable_fraction']*100:.2f}% | host |t| RMS={a['relative_translation']['magnitude_m']['rms']*1000:.3f}mm "
              f"| phase |t| RMS={ps['relative_translation']['magnitude_m']['rms']*1000:.3f}mm | orient RMS={a['relative_orientation']['angle']['rms_deg']:.3f}deg")
