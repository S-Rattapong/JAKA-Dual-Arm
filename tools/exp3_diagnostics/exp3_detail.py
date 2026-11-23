"""Read-only EXP3 (relative pose) drill-down for one run; prints summary + compact series as JSON."""
import json, sys, math, numpy as np
rd = sys.argv[1]
a = json.load(open(rd + "/exp2_analysis.json")); r = json.load(open(rd + "/raw.json"))
P = r["planned"]["samples"]
T = np.array([s["time_from_start_s"] for s in P])
obj = np.array([s["cartesian"]["object_pose"]["translation_m"] for s in P])
R = [np.array(s["cartesian"]["object_pose"]["matrix"])[:3, :3] for s in P]
v_lin = np.r_[0, np.linalg.norm(np.diff(obj, axis=0), axis=1) / np.diff(T)] * 1000  # mm/s
v_ang = [0.0] + [math.degrees(math.acos(max(-1, min(1, (np.trace(R[i].T @ R[i+1]) - 1) / 2)))) / (T[i+1]-T[i]) for i in range(len(R)-1)]
v_ang = np.array(v_ang)
seg = r.get("plan_snapshot", {}).get("segment_durations_s") or []
wp = np.cumsum([0] + list(seg))
out = {"waypoint_times_s": [round(x, 3) for x in wp], "duration_s": r["execution"]["duration_s"]}
def summarize(samples, tkey, name):
    t = np.array([s[tkey] for s in samples]); e = np.array([s["relative_translation_error_m"] for s in samples]) * 1000
    mag = np.linalg.norm(e, axis=1); ang = np.array([s["relative_orientation_error_deg"] for s in samples])
    vl = np.interp(t, T, v_lin); va = np.interp(t, T, v_ang)
    dt = np.r_[np.diff(t), 0]
    res = {"n": len(t), "mag_mean": mag.mean(), "mag_rms": math.sqrt((mag**2).mean()), "mag_p50": np.percentile(mag, 50),
           "mag_p95": np.percentile(mag, 95), "mag_max": mag.max(), "mag_max_t": t[mag.argmax()],
           "axis_mean": e.mean(0).tolist(), "axis_rms": np.sqrt((e**2).mean(0)).tolist(), "axis_maxabs": np.abs(e).max(0).tolist(),
           "ang_mean": ang.mean(), "ang_rms": math.sqrt((ang**2).mean()), "ang_p95": np.percentile(ang, 95), "ang_max": ang.max(), "ang_max_t": t[ang.argmax()],
           "time_above_mm": {k: float(dt[mag > k].sum()) for k in (2, 3, 5, 8)},
           "corr_mag_vs_obj_speed": float(np.corrcoef(mag, vl)[0, 1]), "corr_mag_vs_obj_angvel": float(np.corrcoef(mag, va)[0, 1]),
           "mag_when_moving": float(np.sqrt((mag[vl > 5] ** 2).mean())) if (vl > 5).any() else None,
           "mag_when_slow": float(np.sqrt((mag[vl <= 5] ** 2).mean())) if (vl <= 5).any() else None}
    order = np.argsort(-mag)[:40]; peaks = []
    for i in order:
        if all(abs(t[i] - p["t"]) > 0.4 for p in peaks):
            peaks.append({"t": float(t[i]), "mag": float(mag[i]), "xyz": e[i].round(2).tolist(), "ang": float(ang[i]),
                          "obj_v_mm_s": float(vl[i]), "obj_w_deg_s": float(va[i]),
                          "nearest_wp": int(np.argmin(abs(wp - t[i]))) if len(wp) else None,
                          "dt_to_wp": float(min(abs(wp - t[i]))) if len(wp) else None})
        if len(peaks) == 5: break
    res["peaks"] = peaks
    grid = np.arange(0, out["duration_s"] + 0.001, 0.2)
    res["series"] = [round(float(x), 2) if not np.isnan(x) else None for x in np.interp(grid, t, mag, left=np.nan, right=np.nan)]
    res["series_ang"] = [round(float(x), 3) if not np.isnan(x) else None for x in np.interp(grid, t, ang, left=np.nan, right=np.nan)]
    out[name] = res
    out["grid_s"] = [round(float(x), 1) for x in grid]
    out["obj_speed_series"] = [round(float(x), 1) for x in np.interp(grid, T, v_lin)]
    out["obj_angvel_series"] = [round(float(x), 2) for x in np.interp(grid, T, v_ang)]
summarize(a["samples"], "time_from_start_s", "host")
summarize(a["phase_synchronized"]["samples"], "phase_time_s", "phase")
out["center_consistency_host"] = {k: a["center_consistency"]["translation_magnitude_m"][k] * 1000 for k in ("rms", "max")}
def conv(o):
    if isinstance(o, (np.floating, np.integer)): return o.item()
    raise TypeError
print(json.dumps(out, default=conv))
