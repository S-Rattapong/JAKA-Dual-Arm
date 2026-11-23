"""Read-only: look at raw Port10000 packets around EXP3 spikes.
Flags packets whose joint_actual_position is identical to the previous packet (stale/repeated sample)."""
import json, sys, numpy as np
rd = sys.argv[1]; t0w, t1w = float(sys.argv[2]), float(sys.argv[3])
r = json.load(open(rd + "/raw.json")); a = json.load(open(rd + "/exp2_analysis.json"))
st = r["execution"]["start_time_unix_ns"] / 1e6
print("coverage unique/packets:", a["coverage"]["source_unique_actual_samples"], a["coverage"]["source_packet_counts"])
for side in ("left", "right"):
    P = r["actual_packets"][side]
    dup_a = dup_c = 0; rows = []
    prev = None
    for i, p in enumerate(P):
        t = (p["received_at_ms"] - st) / 1000
        qa = np.array(p["actual_joints_rad"]); qc = np.array(p["controller_joint_position_rad"] or [np.nan]*6)
        da = dc = None
        if prev is not None:
            da = float(np.degrees(np.abs(qa - prev[0]).max())); dc = float(np.degrees(np.abs(qc - prev[1]).max()))
            if da == 0: dup_a += 1
            if dc == 0: dup_c += 1
        if t0w <= t <= t1w:
            rows.append(f"  {side} pkt{i:3d} t={t:7.3f}s dt={(t-prev[2])*1000 if prev else 0:6.1f}ms "
                        f"d_actual={da if da is None else round(da,4)}deg d_ctrl={dc if dc is None else round(dc,4)}deg"
                        + ("  <== ACTUAL REPEATED" if da == 0 else "") + ("  <== CTRL REPEATED" if dc == 0 else ""))
        prev = (qa, qc, t)
    print(f"{side}: packets={len(P)} actual-repeated={dup_a} ctrl-repeated={dup_c}")
    print("\n".join(rows))
print("host EXP3 samples in window (t, |e| mm, xyz mm):")
for s in a["samples"]:
    if t0w <= s["time_from_start_s"] <= t1w:
        e = np.array(s["relative_translation_error_m"]) * 1000
        print(f"  t={s['time_from_start_s']:7.3f} |e|={np.linalg.norm(e):6.2f}  {e.round(2).tolist()}")
