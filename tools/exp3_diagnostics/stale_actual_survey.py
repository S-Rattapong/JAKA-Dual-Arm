"""Read-only survey of 'stale actual' Port10000 packets across all saved runs.

For every packet while the plan moves fast enough (>= 0.12 rad/s), compute
tracking lag = phase(joint_position) - phase(joint_actual_position) on the
frozen plan and its deviation from the run/side median. Reports how often
large positive deviations occur, whether deviations cluster at multiples of
the 8 ms controller cycle, and what precedes/follows them.
usage: stale_actual_survey.py RUNS_DIR OUT_JSON
"""
import json, os, sys
import numpy as np

GRID = 0.0005
runs_dir, out_path = sys.argv[1], sys.argv[2]
summary, devs_all, events = [], [], []
for run in sorted(os.listdir(runs_dir)):
    p = os.path.join(runs_dir, run, "raw.json")
    if not os.path.isfile(p):
        continue
    try:
        raw = json.load(open(p))
        P = raw["planned"]["samples"]; T = np.array([s["time_from_start_s"] for s in P])
        packets = raw["actual_packets"]
        label = json.load(open(os.path.join(runs_dir, run, "manifest.json"))).get("label")
    except Exception:
        continue
    if len(T) < 3 or not isinstance(packets, dict):
        continue
    grid = np.arange(0, T[-1] + 1e-9, GRID)
    row = {"run": run, "label": label}
    for side in ("left", "right"):
        pk = packets.get(side) or []
        if not pk or pk[0].get("controller_joint_position_rad") is None:
            continue
        Q = np.array([s[side]["joints_rad"] for s in P])
        D = np.stack([np.interp(grid, T, Q[:, j]) for j in range(6)], 1)
        sp = np.r_[0, np.linalg.norm(np.diff(D, axis=0), axis=1) / GRID]
        def phase(q, prev):
            lo = max(0, int((prev - .3) / GRID)); hi = min(len(grid), int((prev + 1.0) / GRID) + 1)
            return float(grid[lo + int(np.linalg.norm(D[lo:hi] - q, axis=1).argmin())])
        pc = pa = 0.0; tc, lag, fast = [], [], []
        for x in pk:
            pc = phase(np.array(x["controller_joint_position_rad"]), pc)
            pa = phase(np.array(x["actual_joints_rad"]), pa)
            k = min(int(pc / GRID), len(grid) - 1)
            tc.append(pc); lag.append((pc - pa) * 1000); fast.append(sp[k] >= 0.12 and 0 < pc < T[-1])
        lag = np.array(lag); fast = np.array(fast); tc = np.array(tc)
        if fast.sum() < 10:
            continue
        med = np.median(lag[fast]); dev = lag - med
        devs_all.extend(dev[fast].tolist())
        big = np.where(fast & (dev > 15))[0]
        for i in big:
            spacing_before = (tc[i] - tc[i - 1]) * 1000 if i > 0 else None
            spacing_after = (tc[i + 1] - tc[i]) * 1000 if i + 1 < len(tc) else None
            events.append({"run": run, "side": side, "packet": int(i), "phase_s": float(tc[i]), "dev_ms": float(dev[i]),
                           "next_dev_ms": float(dev[i + 1]) if i + 1 < len(dev) else None,
                           "ctrl_spacing_before_ms": spacing_before, "ctrl_spacing_after_ms": spacing_after})
        row[side] = {"fast_packets": int(fast.sum()), "lag_median_ms": float(med), "lag_std_ms": float(np.std(lag[fast])),
                     "pos_dev_gt15": int((fast & (dev > 15)).sum()), "neg_dev_lt_m15": int((fast & (dev < -15)).sum()),
                     "spacing_ms_all": [float(x) for x in np.diff(tc[fast]) * 1000]}
    if len(row) > 2:
        summary.append(row)
json.dump({"summary": summary, "devs": devs_all, "events": events}, open(out_path, "w"))
print("runs", len(summary), "fast packets", len(devs_all), "events(+>15ms)", len(events))
