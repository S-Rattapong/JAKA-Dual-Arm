"""Read-only: per packet, tracking lag = phase(controller joint_position) - phase(joint_actual_position)
on the frozen plan, for both arms. Lists outliers vs the run median."""
import json, sys, numpy as np
rd = sys.argv[1]
raw = json.load(open(rd + "/raw.json"))
P = raw["planned"]["samples"]; T = np.array([s["time_from_start_s"] for s in P])
st_ms = raw["execution"]["start_time_unix_ns"] / 1e6
fine = np.arange(0, T[-1] + 1e-9, 0.0005)
wp = np.cumsum([0] + raw["plan_snapshot"].get("segment_durations_s", []))
for side in ("left", "right"):
    QF = np.stack([np.interp(fine, T, np.array([p[side]["joints_rad"] for p in P])[:, j]) for j in range(6)], 1)
    spd = np.r_[0, np.linalg.norm(np.diff(QF, axis=0), axis=1) / 0.0005]
    def phase(q, prev):
        lo = max(0, int((prev - .3) / .0005)); hi = min(len(fine), int((prev + 1.0) / .0005))
        d = np.linalg.norm(QF[lo:hi] - q, axis=1); k = int(d.argmin()); return fine[lo + k], d[k]
    pa = pc = 0.0; rows = []
    for i, p in enumerate(raw["actual_packets"][side]):
        pc, rc = phase(np.array(p["controller_joint_position_rad"]), pc)
        pa, ra = phase(np.array(p["actual_joints_rad"]), pa)
        v = spd[int(pc / .0005)] if pc < T[-1] else 0
        rows.append((i, (p["received_at_ms"] - st_ms) / 1000, pc, (pc - pa) * 1000, v, ra))
    R = np.array(rows); mv = R[:, 4] > 0.05  # joint-space speed > ~3 deg/s
    lag = R[mv, 3]; med = np.median(lag)
    print(f"{side}: moving packets={mv.sum()} lag median={med:.1f} ms  p5..p95={np.percentile(lag,5):.1f}..{np.percentile(lag,95):.1f} ms  std={lag.std():.1f}")
    out = [r for r in R[mv] if abs(r[3] - med) > 15]
    for r in out:
        print(f"   pkt{int(r[0]):3d} host_t={r[1]:6.3f}s phase={r[2]:6.3f}s lag={r[3]:6.1f}ms (dev {r[3]-med:+.1f}) jointspeed={np.degrees(r[4]):.1f}deg/s nearest W{int(np.argmin(abs(wp-r[2])))} ({min(abs(wp-r[2])):.2f}s)")
