"""Read-only: estimate each packet's true sample time from its controller-reported joint_position
(phase on the frozen plan, which the controller follows in time), compare with host receive time,
and rebuild EXP3 relative error from per-packet controller-time pairing."""
import json, sys, math, numpy as np
rd = sys.argv[1]
r = json.load(open(rd + "/raw.json")); a = json.load(open(rd + "/exp2_analysis.json"))
P = r["planned"]["samples"]; T = np.array([s["time_from_start_s"] for s in P])
st = r["execution"]["start_time_unix_ns"] / 1e6
fine = np.arange(0, T[-1] + 1e-9, 0.0005)
QF = {s: np.stack([np.interp(fine, T, np.array([p[s]["joints_rad"] for p in P])[:, j]) for j in range(6)], 1) for s in ("left", "right")}
spd = {s: np.r_[0, np.linalg.norm(np.diff(QF[s], axis=0), axis=1)] for s in QF}
res = {}
for side in ("left", "right"):
    rows = []; prev = 0.0
    for p in r["actual_packets"][side]:
        q = np.array(p["controller_joint_position_rad"]); lo = max(0, int((prev - .3) / .0005)); hi = min(len(fine), int((prev + 1.0) / .0005))
        d = np.linalg.norm(QF[side][lo:hi] - q, axis=1); k = int(d.argmin()); tau = fine[lo + k]; prev = tau
        rows.append(((p["received_at_ms"] - st) / 1000, tau, p["actual_joints_rad"], spd[side][lo + k] > 1e-6))
    res[side] = rows
    th = np.array([x[0] for x in rows]); ta = np.array([x[1] for x in rows]); mv = np.array([x[3] for x in rows])
    off = (th - ta)[mv]; base = np.median(off)
    dt_host = np.diff(th)[mv[1:]] * 1000; dt_ctrl = np.diff(ta)[mv[1:]] * 1000
    print(f"{side}: moving pkts={mv.sum()} host-minus-controller time: median={base*1000:.1f}ms "
          f"spread(p5..p95)={np.percentile(off-base,5)*1000:+.1f}..{np.percentile(off-base,95)*1000:+.1f}ms "
          f"std={np.std(off)*1000:.1f}ms max|dev|={np.abs(off-base).max()*1000:.1f}ms")
    hist = np.round(dt_ctrl / 8) * 8
    vals, cnt = np.unique(hist, return_counts=True)
    print(f"   controller sample spacing (rounded to 8 ms): " + ", ".join(f"{int(v)}ms x{c}" for v, c in zip(vals, cnt) if c > 2))
    print(f"   host arrival spacing p5/p50/p95: {np.percentile(dt_host,5):.0f}/{np.percentile(dt_host,50):.0f}/{np.percentile(dt_host,95):.0f} ms")
# rebuild EXP3 with controller-time pairing using production FK model
sys.path.insert(0, "/home/newtonrattapong/jaka_ws")
from dual_arm_app.backend import experimental_rigid_grasp as g
src = open(g.__file__).read()
print("FK helpers:", [n for n in dir(g) if "fk" in n.lower() or "Urdf" in n][:10])
