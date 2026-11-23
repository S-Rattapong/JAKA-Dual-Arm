"""Summarize stale_actual_survey.py output. usage: stale_actual_report.py SURVEY_JSON

Excludes runs where plan-phase matching is not meaningful: EXP4 out-and-back paths
(the path revisits the same joint states, so phase search can jump) and runs that
were never executed (DISARMED; no motion to match)."""
import json, os, sys
import numpy as np
d = json.load(open(sys.argv[1]))
runs_dir = "/home/newtonrattapong/jaka_ws/dual_arm_app/experiment_runs"
def usable(r):
    if (r["label"] or "").startswith("EXP4"):
        return False
    m = json.load(open(os.path.join(runs_dir, r["run"], "manifest.json")))
    return m.get("state") == "CLOSED" and "DISARMED" not in json.dumps(m.get("result", {}))
keep = [r for r in d["summary"] if usable(r)]
names = {r["run"] for r in keep}
ev = [e for e in d["events"] if e["run"] in names]
fast = sum(r[s]["fast_packets"] for r in keep for s in ("left", "right") if s in r)
neg = sum(r[s]["neg_dev_lt_m15"] for r in keep for s in ("left", "right") if s in r)
print(f"usable runs={len(keep)} fast packets={fast} | +>15ms events={len(ev)} ({100*len(ev)/fast:.2f}%) | -<-15ms={neg} ({100*neg/fast:.2f}%)")
lagmed = [r[s]["lag_median_ms"] for r in keep for s in ("left", "right") if s in r]
lagstd = [r[s]["lag_std_ms"] for r in keep for s in ("left", "right") if s in r]
print(f"per-run tracking-lag median range {min(lagmed):.1f}..{max(lagmed):.1f} ms; per-run std median {np.median(lagstd):.1f} ms")
pos = np.array([e["dev_ms"] for e in ev])
print(f"event deviation: median={np.median(pos):.1f} p90={np.percentile(pos,90):.1f} max={pos.max():.1f} ms")
h, edges = np.histogram(pos, bins=np.arange(14, min(pos.max(), 120) + 4, 4))
for c, a in zip(h, edges[:-1]):
    print(f"  {a:4.0f}-{a+4:<4.0f}ms {'#'*int(round(40*c/max(h.max(),1)))} {c}")
m8 = np.abs(((pos + 4) % 8) - 4)
print(f"distance to nearest multiple of 8 ms: median={np.median(m8):.2f} ms (uniform expectation 2.00 ms); within 1 ms: {(m8<1).mean()*100:.0f}% (uniform 25%)")
nxt = np.array([e["next_dev_ms"] for e in ev if e["next_dev_ms"] is not None])
print(f"next packet deviation: median={np.median(nxt):.1f} ms, |next|<10 ms: {(np.abs(nxt) < 10).mean()*100:.0f}%, next also >15: {(nxt>15).mean()*100:.0f}%")
sb = np.array([e["ctrl_spacing_before_ms"] for e in ev if e["ctrl_spacing_before_ms"] is not None])
allsp = np.array([x for r in keep for s in ("left", "right") if s in r for x in r[s]["spacing_ms_all"]])
for lo, hi in ((0, 76), (76, 100), (100, 124), (124, 400)):
    a = ((allsp >= lo) & (allsp < hi)).sum(); e_ = ((sb >= lo) & (sb < hi)).sum()
    print(f"   controller spacing before {lo:3d}-{hi:<3d} ms: events {e_:3d} / packets {a:5d} -> {100*e_/max(a,1):.2f}%")
print("events by side:", {s: sum(1 for e in ev if e["side"] == s) for s in ("left", "right")})
for r in keep:
    n = sum(r[s]["fast_packets"] for s in ("left", "right") if s in r); k = sum(r[s]["pos_dev_gt15"] for s in ("left", "right") if s in r)
    print(f"   {r['label'][:24]:24s} {n:5d} {k:4d} {100*k/max(n,1):5.1f}%")
