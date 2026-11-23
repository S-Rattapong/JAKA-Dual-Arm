"""Read-only sensitivity test: re-run the PRODUCTION host-time EXP3 analyzer (_analyze_host_time)
on an in-memory copy of raw.json whose packet timestamps are replaced by the sample time inferred
from each packet's controller-reported joint_position (phase on the frozen plan).
Nothing is written to the run directory."""
import copy, json, sys, numpy as np
sys.path.insert(0, "/home/newtonrattapong/jaka_ws")
from dual_arm_app.backend.experimental_rigid_grasp import _analyze_host_time
from dual_arm_app.backend.experimental_infrastructure import EXP2_URDF_PATH
rd = sys.argv[1]
raw = json.load(open(rd + "/raw.json")); man = json.load(open(rd + "/manifest.json"))
P = raw["planned"]["samples"]; T = np.array([s["time_from_start_s"] for s in P])
st_ms = raw["execution"]["start_time_unix_ns"] / 1e6
fine = np.arange(0, T[-1] + 1e-9, 0.0005)
def summary(res, label):
    m = res["relative_translation"]["magnitude_m"]; o = res["relative_orientation"]["angle"]
    print(f"{label:38s} usable={res['coverage']['usable_fraction']*100:6.2f}%  |e| RMS={m['rms']*1000:6.3f} mm  "
          f"max={m['max']*1000:6.2f} mm @ {m['max_location']['time_from_start_s']:.2f}s  orient RMS={o['rms_deg']:.3f} deg max={o['max_deg']:.2f}")
summary(_analyze_host_time(raw, man, EXP2_URDF_PATH), "ORIGINAL host receive time (recomputed)")
DROP = {"left": {37, 67, 118, 157}, "right": {32, 116}}
for label, per_arm, drop in (("CONTROLLER-INFERRED time (common lat.)", False, False), ("CONTROLLER-INFERRED + drop 6 lag-outlier pkts", False, True), ("HOST time + drop 6 lag-outlier pkts", None, True)):
    mod = copy.deepcopy(raw)
    for side in ("left", "right"):
        QF = np.stack([np.interp(fine, T, np.array([p[side]["joints_rad"] for p in P])[:, j]) for j in range(6)], 1)
        pk = mod["actual_packets"][side]; prev = 0.0; taus = []; moving = []
        for p in pk:
            q = np.array(p["controller_joint_position_rad"]); lo = max(0, int((prev - .3) / .0005)); hi = min(len(fine), int((prev + 1.0) / .0005))
            k = int(np.linalg.norm(QF[lo:hi] - q, axis=1).argmin()); tau = fine[lo + k]; prev = tau
            taus.append(tau); moving.append(0.002 < tau < T[-1] - 0.002)
        host = np.array([(p["received_at_ms"] - st_ms) / 1000 for p in pk]); taus = np.array(taus); moving = np.array(moving)
        lat = 0.090
        new_t = host.copy() if per_arm is None else np.where(moving, taus + lat, host)  # stationary packets keep host time (joints constant there)
        for i in range(1, len(new_t)):            # keep strictly increasing
            if new_t[i] <= new_t[i - 1]: new_t[i] = new_t[i - 1] + 1e-4
        mono0 = pk[0]["received_monotonic_ns"] - int((host[0]) * 1e9)
        for p, t in zip(pk, new_t):
            p["received_at_ms"] = st_ms + t * 1000
            p["received_monotonic_ns"] = int(mono0 + t * 1e9)
        if drop:
            mod["actual_packets"][side] = [p for i, p in enumerate(pk) if i not in DROP[side]]
    summary(_analyze_host_time(mod, man, EXP2_URDF_PATH), label)
