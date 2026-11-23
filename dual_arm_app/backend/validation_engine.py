"""Offline validation math. All lengths inside transforms are metres, angles radians."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

try:
    from .world_frame_calibration import normalize_calibration_data
    from .experimental_rigid_grasp import UrdfFkModel, JOINT_NAMES, _transform
except ImportError:
    from world_frame_calibration import normalize_calibration_data
    from experimental_rigid_grasp import UrdfFkModel, JOINT_NAMES, _transform

SIDES = ("left", "right")
APP = Path(__file__).resolve().parents[1]
METHODS = {"V1": "FK cross-check", "V2": "Host time shift", "V3": "Surface gap",
           "V4": "Single-arm jig distance", "V5": "Common-point agreement"}
METRICS = {"V1": {"position_norm":"mm", "orientation":"deg"},
           "V2": {"abs_delta_tau":"ms", "abs_tau_left":"ms", "abs_tau_right":"ms"},
           "V3": {"abs_gap_error":"mm"}, "V4": {"abs_distance_error":"mm"}, "V5": {"point_norm":"mm"}}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def number(value, label):
    require(isinstance(value, (float, int)) and not isinstance(value, bool) and np.isfinite(value), label)
    return float(value)


def vec(value, n):
    require(isinstance(value, (list, tuple)) and len(value) == n, f"Expected {n} finite components")
    return np.array([number(x, "Invalid vector component") for x in value])


def stats(values):
    a = np.asarray(values)
    return {"mean": float(a.mean()), "rms": float(np.sqrt(np.mean(a*a))), "max": float(a.max())}


def snapshot():
    urdf = (APP / "web/assets/dual_jaka_a12_web.urdf").read_bytes()
    cal = (APP / "config/world_frame_calibration.json").read_bytes()
    metadata = (APP / "web/assets/dual_jaka_a12_web.metadata.json").read_bytes()
    result = {}
    for key, data in (("urdf", urdf), ("calibration", cal), ("model_metadata", metadata)):
        result[key] = data.decode()
        result[key+"_sha256"] = hashlib.sha256(data).hexdigest()
    return result


def transform(d):
    return np.array(_transform(vec(d["translation_m"], 3), vec(d["rotation_rpy_rad"], 3)))


class Geometry:
    def __init__(self, snap):
        for key in ("urdf", "calibration", "model_metadata"):
            require(hashlib.sha256(snap[key].encode()).hexdigest() == snap[key+"_sha256"], "Snapshot hash mismatch")
        self.model = UrdfFkModel("snapshot.urdf", content=snap["urdf"].encode())
        self.cal = normalize_calibration_data(json.loads(snap["calibration"]))
        require(json.loads(snap["model_metadata"])["calibration_revision"] == self.cal["revision"],
                "Model metadata calibration revision mismatch")
        require(self.cal["tcp_tool_contract"]["status"] == "TOOL0_J6_KINEMATICALLY_VERIFIED",
                "Missing verified J6 model contract")
        self.base = {}
        for side in SIDES:
            link = side+"_base_link"
            self.base[side] = transform(self.cal["transforms"]["world_to_"+side+"_base"])
            require(self.model.joints_by_child[link]["parent"] == "world" and
                    np.allclose(self.model.fk(link, {}), self.base[side], atol=1e-10, rtol=0), "Calibration/model disagreement")
            require(self.cal["tcp_tool_contract"]["planning_tip_frames"][side] == side+"_J6", "Unknown planning tip")

    def pose(self, side, q):
        require(side in SIDES, "Select arm")
        return np.array(self.model.fk(side+"_J6", dict(zip(JOINT_NAMES[side], vec(q, 6)))))


def tool_transform(definition):
    frame = definition.get("tool_frame")
    require(frame in ("J6", "mounted"), "MISSING_INFO: select the tool coordinate frame")
    if frame == "J6":
        return np.eye(4)
    require(definition.get("tool_id"), "Missing mounted tool ID")
    return transform(definition["j6_from_tool"])


def crosscheck(g, side, packet, c):
    m = c.get("mapping", {}).get(side, {})
    require(m.get("verified") is True and m.get("source") and m.get("tool_id") and m.get("frame_id")
            and m.get("model_revision") == g.cal["revision"], "MISSING_INFO: verify TCP field, Tool, frame and model revision")
    field = m["field"]
    require(field in ("controller_actual_position_raw", "controller_position_raw"), "Unknown controller field")
    require(packet.get("mapping") == "port10000_joint_actual_position_deg_to_rad", "MISSING_INFO: actual joint source / units")
    validity = packet["field_validation"]
    require(validity.get(field[11:-4]) == "VALID" and validity.get("joint_actual_position") == "VALID",
            "Invalid same-packet fields")
    raw = vec(packet[field], 6)
    require(m["length_unit"] in ("m", "mm") and m["angle_unit"] in ("deg", "rad") and
            m["orientation"] == "RzRyRx", "MISSING_INFO: units or RPY convention")
    ctrl = np.array(_transform(raw[:3]*(0.001 if m["length_unit"] == "mm" else 1),
                               np.deg2rad(raw[3:]) if m["angle_unit"] == "deg" else raw[3:]))
    frame = m["frame"]
    require(frame in ("world", "base", "user"), "Unknown controller frame")
    if frame == "base":
        ctrl = g.base[side] @ ctrl
    elif frame == "user":
        ctrl = transform(m["world_from_user"]) @ ctrl
    fk = g.pose(side, packet["actual_joints_rad"]) @ transform(m["j6_from_tcp"])
    delta = (fk[:3, 3] - ctrl[:3, 3])*1000
    return {"side": side, "received_monotonic_ns": packet["received_monotonic_ns"],
            "xyz_mm": delta.tolist(), "error_mm": float(np.linalg.norm(delta)),
            "orientation_deg": float(np.rad2deg(Rotation.from_matrix(fk[:3,:3].T @ ctrl[:3,:3]).magnitude()))}


def gap(poses, c, trial):
    points, normals = [], []
    for side in SIDES:
        surface = c["surfaces"][side]
        require(surface["source"], "Missing surface definition source")
        pose = poses[side] @ tool_transform(surface)
        points.append(pose[:3,:3] @ vec(surface["point_m"], 3) + pose[:3,3])
        normal = pose[:3,:3] @ vec(surface["normal"], 3)
        require(np.linalg.norm(normal) > 1e-12, "Zero surface normal")
        normals.append(normal/np.linalg.norm(normal))
    angle = float(np.rad2deg(np.arccos(np.clip(abs(normals[0] @ normals[1]), 0, 1))))
    tolerance = number(c["parallel_deg"], "Missing parallel tolerance")
    require(0 <= tolerance < 90 and angle <= tolerance + 1e-9, f"Planes differ by {angle:g} deg")
    fk = float((points[1] - points[0]) @ normals[0]*1000)
    row = {"fk_gap_mm": fk, "parallel_deg": angle, "planned_mm": number(trial["planned_mm"], "Missing planned gap")}
    gauge = c["gauge"]
    require(gauge["brand"] and gauge["type"] and number(gauge["resolution_mm"], "Gauge resolution") > 0 and
            number(gauge["uncertainty_mm"], "Gauge uncertainty") >= 0, "Missing gauge provenance")
    if trial.get("interval_mm") is not None:
        require(trial.get("measured_mm") is None, "Choose one measurement mode")
        lo, hi = vec(trial["interval_mm"], 2)
        require(lo <= hi, "Reversed gauge interval")
        return row | {"error_interval_mm": [lo-fk, hi-fk]}
    return row | {"error_mm": number(trial["measured_mm"], "Missing independent gauge reading")-fk}


def shift(plan, packets, side, origin_ns, c):
    t = np.asarray(plan["t"], dtype=float)
    q = np.asarray(plan[side], dtype=float)
    require(3 <= len(t) <= 30000 and q.shape == (len(t), 6) and np.isfinite(q).all() and
            np.isfinite(t).all() and np.all(np.diff(t)>0), "Invalid original plan")
    lo, hi, step = [number(c[k], k) for k in ("tau_min_ms", "tau_max_ms", "tau_step_ms")]
    span = number(c.get("gap_ms", 250), "gap_ms")/1000
    require(span > 0 and lo <= 0 <= hi and lo < hi and step > 0 and (hi-lo)/step <= 400, "Invalid shift grid")
    grid = np.unique(np.r_[np.arange(lo, hi, step), 0., hi])
    require(3 <= len(packets) <= 30000, "Missing or oversized packet series")
    # Both execution origin and packet wall timestamps are on the host Unix clock.
    wall_ms = np.array([number(p["received_at_ms"], "Missing host wall time") for p in packets])
    times = (wall_ms-number(origin_ns, "Missing host start")/1e6)/1000
    clock = "host_wall_receive"
    if all(p.get("received_monotonic_ns") is not None for p in packets):
        stamps = np.array([p["received_monotonic_ns"] for p in packets], dtype=np.int64)
        elapsed = (stamps-stamps[0])/1e9
        require(np.max(np.abs(elapsed-(wall_ms-wall_ms[0])/1000)) <= .010,
                "Host clock discontinuity exceeds 10 ms")
        times = times[0] + elapsed
        clock = "host_monotonic_receive_anchored_to_first_packet_wall_time"
    actual = np.array([vec(p["actual_joints_rad"], 6) for p in packets])
    require(np.all(np.diff(times)>0), "Non-increasing host receive clock")
    # Evaluate at actual observations only. No synthesized actual sample spans an outage.
    # Remove plan support holes for EVERY shift before comparing any RMSE.
    mask = np.ones(len(times), dtype=bool)
    for tau in grid:
        u = times-tau/1000
        idx = np.searchsorted(t, u).clip(1, len(t)-1)
        mask &= (u>=t[0]) & (u<=t[-1]) & ((t[idx]-t[idx-1]<=span) | (t[idx]==u) | (t[idx-1]==u))
    times, actual = times[mask], actual[mask]
    require(len(times)>=3, "Insufficient common shift support")
    def predicted(tau):
        return np.column_stack([np.interp(times-tau/1000, t, a) for a in q.T])
    curve = np.array([np.sqrt(np.mean((actual-predicted(x))**2)) for x in grid])
    i = int(curve.argmin())
    best = predicted(grid[i])
    warnings = []
    if i in (0, len(grid)-1):
        warnings.append("BOUNDARY_MINIMUM")
    if np.ptp(curve) <= max(1e-9, 0.001*curve.max()):
        warnings.append("FLAT_CURVE")
    # Preserve outages in the plotted overlays.
    take = np.linspace(0, len(times)-1, min(500, len(times)), dtype=int)
    return {"tau_ms": grid[i], "grid_ms": grid.tolist(), "rmse_rad": curve.tolist(),
            "before_rad": curve[np.where(grid==0)[0][0]], "after_rad": curve[i],
            "joint_rmse_before_rad": np.sqrt(np.mean((actual-predicted(0))**2, axis=0)).tolist(),
            "joint_rmse_rad": np.sqrt(np.mean((actual-best)**2, axis=0)).tolist(),
            "clock": clock, "common_packets": len(times), "packet_gaps": int(sum(np.diff(times)>span)), "warnings": warnings,
            "overlay": {"t": times[take].tolist(), "actual": actual[take].tolist(),
                        "plan": predicted(0)[take].tolist(), "shifted": best[take].tolist(), "residual_rad": (actual-best)[take].tolist(),
                        "segment": np.cumsum(np.r_[False,np.diff(times)>span])[take].tolist()}}


def analyze(session):
    method, c = session["method"], session["config"]
    rows, issues, values, angles = [], [], [], []
    criterion_issues, training_issues, training_rows = [], [], []
    unknown_reference_uncertainty_jigs = set()
    pairs, repeats, point_results = defaultdict(dict), defaultdict(list), defaultdict(list)
    active = [x for x in session["samples"] if not x.get("excluded")]
    try:
        geometry = Geometry(session["snapshot"]) if method != "V2" else None
        if method in ("V3", "V5"):
            require(geometry.cal["physical_calibration"] == "CALIBRATED" and geometry.cal["source"].get("method"),
                    "Physical world calibration and source required")
    except (TypeError, KeyError, AttributeError, ValueError) as exc:
        issues.append(f"MISSING_INFO: {exc}")
        active = []
    for sample in active:
        label = sample["key"]
        try:
            t = sample["trial"]
            packets = sample["packets"]
            if sample.get("raw_sha256") and c.get("criterion"):
                # Creation of an import session can postdate the experiment.
                stamp = c["criterion"].get("declared_at_ms")
                first = [p.get("received_at_ms") for series in packets.values() for p in series]
                if not (stamp is not None and first and all(isinstance(v,(int,float)) for v in first)
                        and stamp <= min(first)):
                    criterion_issues.append(f"{label}: missing evidence of a criterion predating this run")
            if method == "V1":
                require(sample.get("model_revision") == geometry.cal["revision"], "MISSING_INFO: source model compatibility")
                for side, series in packets.items():
                    rejected = defaultdict(int)
                    for p in series:
                        try:
                            row = crosscheck(geometry, side, p, c)
                            rows.append({"sample": label, **row})
                            values.append(row["error_mm"])
                            angles.append(row["orientation_deg"])
                        except (TypeError, KeyError, AttributeError, ValueError) as exc:
                            rejected[str(exc)] += 1
                    if not series:
                        issues.append(f"{label} {side}: empty packet series")
                    issues.extend(f"{label} {side}: {why} ({count} packets)" for why,count in rejected.items())
            elif method == "V2":
                row = {side: shift(sample["plan"], packets[side], side, sample["start_ns"], c) for side in SIDES}
                row["delta_tau_ms"] = row["right"]["tau_ms"] - row["left"]["tau_ms"]
                rows.append({"sample": label, **row})
                values.append(abs(row["delta_tau_ms"]))
                for side in SIDES:
                    issues.extend(f"{label} {side}: {w}" for w in row[side]["warnings"])
            else:
                poses = {s: geometry.pose(s, np.median([vec(p["actual_joints_rad"], 6) for p in ps], axis=0).tolist())
                         for s, ps in packets.items()}
                if method == "V3":
                    row = gap(poses, c, t)
                    rows.append({"sample": label, **t, **row})
                    if "error_mm" in row:
                        values.append(abs(row["error_mm"]))
                    continue
                side = t["side"]
                probe = c["probes"][side]
                require(probe["source"], "Probe offset source required")
                point = (poses[side] @ tool_transform(probe) @ np.r_[vec(probe["point_m"], 3), 1])[:3]*1000
                role = t.get("role", "validation")
                group = (t["jig"], side, t["repeat"]) if method == "V4" else (role, t["socket"], t["repeat"])
                member = t["socket"] if method == "V4" else side
                require(member not in pairs[group], "Duplicate active pair member")
                pairs[group][member] = (point, label)
                repeats[group[:2]+(member,)].append(point)
        except (TypeError, KeyError, AttributeError, ValueError, IndexError) as exc:
            target_issues = training_issues if method=="V5" and sample["trial"].get("role")=="training" else issues
            target_issues.append(f"{label}: {exc}")

    for group, members in pairs.items():
        try:
            if method == "V4":
                jig, side, repeat = group
                ref = c["jigs"][jig]
                require(ref.get("source"), "Missing jig measurement source")
                uncertainty = ref.get("uncertainty_mm")
                if uncertainty is None:
                    # Unknown != zero. Geometry can be characterized, but an
                    # acceptance claim must be blocked until uncertainty exists.
                    unknown_reference_uncertainty_jigs.add(jig)
                else:
                    require(number(uncertainty, "Instrument uncertainty") >= 0,
                            "Invalid jig measurement uncertainty")
                a, b = (members[k][0] for k in ("A","B"))
                dist = float(np.linalg.norm(a-b))
                reference = number(ref["distance_mm"], "Missing jig distance")
                require(reference > 0, "Invalid jig distance")
                error = dist-reference
                rows.append({"jig": jig, "side": side, "repeat": repeat, "distance_mm": dist, "error_mm": error,
                             "samples": [members[k][1] for k in ("A","B")]})
                values.append(abs(error))
            else:
                role, socket, repeat = group
                l, r = (members[s][0] for s in SIDES)
                xyz = l-r
                err = float(np.linalg.norm(xyz))
                target_rows = training_rows if role == "training" else rows
                target_rows.append({"role": role, "socket": socket, "repeat": repeat, "xyz_mm": xyz.tolist(), "error_mm": err,
                             "samples": [members[s][1] for s in SIDES]})
                if role == "validation":
                    values.append(err)
                    point_results[socket].append((l, err))
        except (KeyError, ValueError, TypeError) as exc:
            target_issues = training_issues if method=="V5" and group[0]=="training" else issues
            target_issues.append(f"{group}: incomplete pair or reference ({exc})")
    extra = {}
    if method == "V4":
        extra["reference_uncertainty"] = {
            "missing_jig_ids": sorted(unknown_reference_uncertainty_jigs),
            "status": ("UNKNOWN" if unknown_reference_uncertainty_jigs else "PROVIDED"),
        }
        extra["repeat_dispersion"] = [{"group": list(group), "count": len(ps),
                                      "radial_rms_mm": float(np.sqrt(np.mean(np.sum((ps-np.mean(ps, axis=0))**2,axis=1)))) if len(ps)>1 else None}
                                     for group, ps in repeats.items()]
        for jig in c.get("jigs", {}):
            for i in range(1, c["repetitions"]+1):
                if (jig,c["side"],i) not in pairs:
                    issues.append(f"{jig} trial {i}: missing")
    if method == "V5":
        extra["training"] = {"rows":training_rows,"issues":training_issues}
        extra["by_point_mm"] = {k: stats([v for _,v in ps]) for k,ps in point_results.items()}
        centers = [np.mean([p for p,_ in ps],axis=0) for ps in point_results.values()]
        if len(centers)<4 or np.linalg.matrix_rank(centers-np.mean(centers,axis=0),tol=0.1)<3:
            issues.append("Need four spatially distributed noncoplanar validation points")
        for socket in c.get("sockets", []):
            for i in range(1,c["repetitions"]+1):
                if ("validation",socket,i) not in pairs:
                    issues.append(f"{socket} trial {i}: missing")
    if method == "V2" and rows:
        extra["run_summary_ms"] = {}
        for key in (*SIDES,"delta_tau_ms"):
            a = [r[key]["tau_ms"] if key in SIDES else r[key] for r in rows]
            extra["run_summary_ms"][key] = {"mean": float(np.mean(a)), "sd": float(np.std(a,ddof=1)) if len(a)>1 else None}
    if angles:
        extra["signed_xyz_mm"] = {axis:stats([r["xyz_mm"][i] for r in rows]) for i,axis in enumerate("xyz")}
        extra["orientation_deg"] = stats(angles)
        extra["inter_arm_relative_pose"] = "UNKNOWN: same-frame pairing unverified"
    if method == "V3":
        groups = defaultdict(set)
        for r in rows:
            groups[(r["pose"],r["direction"])].add(r["repeat"])
        if any(len(v)<c["repetitions"] for v in groups.values()):
            issues.append("Missing pose/direction repetitions")
    if not rows:
        issues.append("No evaluable trials")
    criterion = c.get("criterion")
    metric = (criterion or {}).get("metric", next(iter(METRICS[method])))
    if metric == "orientation":
        values = angles
    elif metric in ("abs_tau_left", "abs_tau_right"):
        side = metric.split("_")[-1]
        values = [abs(r[side]["tau_ms"]) for r in rows]
    acceptance = "CANNOT_EVALUATE" if issues else "MEASURED_NO_CRITERION"
    if method == "V4" and unknown_reference_uncertainty_jigs:
        if criterion:
            criterion_issues.append(
                "Unknown reference Jig uncertainty; no defensible PASS/FAIL"
            )
    if criterion and criterion_issues and not issues:
        acceptance = "CANNOT_EVALUATE"
    if criterion and not issues and not criterion_issues:
        limit = criterion["value"]
        bounds = [(x,x) for x in values]
        for r in rows:
            if "error_interval_mm" in r:
                lo, hi = r["error_interval_mm"]
                bounds.append((0 if lo<=0<=hi else min(abs(lo),abs(hi)), max(abs(lo),abs(hi))))
        if bounds:
            acceptance = "FAIL" if any(lo>limit for lo,_ in bounds) else (
                "PASS" if all(hi<=limit for _,hi in bounds) else "CANNOT_EVALUATE")
    return {"method": method, "rows": rows, "issues": issues, "data_status": "CANNOT_EVALUATE" if issues else "MEASURED", "enough_data": bool(rows) and not issues,
            "acceptance": acceptance, "criterion": criterion, "absolute_error": stats(values) if values else None,
            "measurement_uncertainty": c.get("measurement_uncertainty") or "UNKNOWN",
            "unit": METRICS[method][metric], "criterion_issues": criterion_issues, "metric": metric,
            "limitation": {"V1":"Model consistency only. No external physical accuracy evidence.",
                           "V2":"Effective host-receipt delay including transport. Grid resolution and packet coverage limit timing precision.",
                           "V3":"Accuracy depends on surface definitions, plane alignment and independent gauge uncertainty.",
                           "V4":("Accuracy depends on reference measurement, probe center and seating repeatability."
                                 + (" Reference jig uncertainty is UNKNOWN; descriptive results only, not PASS/FAIL."
                                    if unknown_reference_uncertainty_jigs else "")),
                           "V5":"Common-point agreement uses the saved world calibration and probe definitions."}[method], **extra}
