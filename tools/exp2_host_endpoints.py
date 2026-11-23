"""Estimate threshold-crossing intervals on the HOST receive clock from EXP raw.json.

Outputs R-L interval bounds (ms). These are host observation intervals only.
Defaults are diagnostic thresholds, to be calibrated against stationary feedback.
"""
import argparse
import csv
import json
from math import degrees, isfinite
from pathlib import Path


def endpoints(times, joints, start, goal, angle, speed, hold, max_gap):
    if len(times) != len(joints) or any(len(q) != 6 for q in [start, goal, *joints]):
        raise ValueError("Expected one six-joint vector per timestamp")
    if not all(isfinite(v) for q in [start, goal, *joints] for v in q):
        raise ValueError("Invalid joint angle")
    if not all(isfinite(t) for t in times) or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Expected strictly increasing host timestamps")

    def deviation(q, ref):
        return max(abs(degrees(a - b)) for a, b in zip(q, ref))

    moved = [deviation(q, start) > angle for q in joints]
    onset = None
    first = None
    for i in range(1, len(times)):
        if not moved[i] or times[i] - times[i - 1] > max_gap:
            first = None
        elif not moved[i - 1]:
            first = i
        if first is not None and times[i] - times[first] >= hold:
            onset = [times[first - 1], times[first]]
            break

    # Qualify a final continuous sequence inside the goal band and below speed.
    # The first quiet interval can only bracket settling with the preceding sample.
    settle = None
    first_quiet = None
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        quiet = (dt <= max_gap and deviation(joints[i], goal) <= angle
                 and deviation(joints[i], joints[i - 1]) * 1000 / dt <= speed)
        if not quiet:
            first_quiet = None
        elif first_quiet is None:
            first_quiet = i
    if onset and first_quiet is not None and times[first_quiet - 1] > onset[1]:
        if times[-1] - times[first_quiet] >= hold and times[first_quiet] - times[first_quiet - 1] <= max_gap:
            settle = [times[first_quiet - 1], times[first_quiet]]
    return onset, settle


def difference(left, right):
    if left is None or right is None:
        return None
    return [right[0] - left[1], right[1] - left[0]]


def classify(bounds):
    if bounds is None:
        return "INCOMPLETE"
    if bounds[0] > 0:
        return "RIGHT_LATER"
    if bounds[1] < 0:
        return "RIGHT_EARLIER"
    return "INCLUDES_ZERO"


def summarize(values):
    return {"count": len(values), "max": max(values, default=None),
            "mean": sum(values) / len(values) if values else None}


def analyze(raw, angle=None, speed=None, hold=None, max_gap=None):
    evidence = raw.get("endpoint_evidence", {})
    if evidence and evidence.get("schema_version") != "exp2-host-endpoints/v1":
        raise ValueError("Unsupported endpoint evidence version")
    policy = dict(angle_deg=.05, speed_deg_s=.5, hold_ms=100., max_gap_ms=250.,
                  pre_ms=1000., post_ms=1000.)
    policy.update(evidence.get("policy", {}))
    for key, value in zip(("angle_deg", "speed_deg_s", "hold_ms", "max_gap_ms"),
                          (angle, speed, hold, max_gap)):
        if value is not None:
            policy[key] = value
    if any(not isfinite(x) or x <= 0 for x in policy.values()):
        raise ValueError("Thresholds must be finite and positive")
    plan = raw["planned"]["samples"]
    bound = evidence.get("bind_observed_ns")
    terminal = evidence.get("terminal_observed_ns")
    result = {"run_id": raw.get("run_id"), "terminal_status": (raw.get("result") or {}).get("status"),
              "clock": "host monotonic receive/parse", "positive_difference": "right later",
              "policy": policy, "capture_outcome": evidence.get("outcome", "LEGACY"),
              "driver_deadline_send_queue_and_controller_drops": "UNMEASURED", "arms": {}}
    for side in ("left", "right"):
        rows = (evidence["packets"] if evidence else raw.get("actual_packets", {})).get(side, [])
        valid = [p for p in rows if type(p.get("received_monotonic_ns")) is int]
        ns = [p["received_monotonic_ns"] for p in valid]
        times = [n / 1e6 for n in ns]
        joints = [p["actual_joints_rad"] for p in valid]
        start = plan[0][side]["joints_rad"]
        gaps = [(b-a)/1e6 for a, b in zip(ns, ns[1:])]
        seq = [p.get("packet_sequence_index") for p in valid]
        resets = sum(b <= a for a,b in zip(seq, seq[1:]) if type(a) is int and type(b) is int)
        jumps = sum(b > a+1 for a,b in zip(seq, seq[1:]) if type(a) is int and type(b) is int)
        pre = [i for i,n in enumerate(ns) if bound is not None and n <= bound]
        noise = [max(abs(degrees(a-b)) for a,b in zip(joints[i], start)) for i in pre]
        rates = [max(abs(degrees(a-b)) for a,b in zip(joints[i], joints[i-1]))*1000/gaps[i-1]
                 for i in pre if i and gaps[i-1] > 0]
        reasons = []
        if len(valid) != len(rows) or evidence.get("invalid_receive_clock", {}).get(side, 0):
            reasons.append("INVALID_CLOCK")
        if any(dt <= 0 for dt in gaps):
            reasons.append("DUPLICATE_OR_REVERSED_CLOCK")
        if resets or jumps:
            reasons.append("LOCAL_SEQUENCE_DISCONTINUITY")
        if not pre or bound - ns[pre[0]] < policy["pre_ms"]*1e6:
            reasons.append("SHORT_BASELINE")
        if any(x > policy["angle_deg"] for x in noise) or any(x > policy["speed_deg_s"] for x in rates):
            reasons.append("BASELINE_OUTSIDE_THRESHOLD")
        onset = settle = None
        if not reasons:
            onset, settle = endpoints(times, joints, start, plan[-1][side]["joints_rad"],
                                      policy["angle_deg"], policy["speed_deg_s"],
                                      policy["hold_ms"], policy["max_gap_ms"])
            if onset and terminal is not None and onset[1] > terminal / 1e6:
                reasons.append("START_AFTER_TERMINAL")
                onset = None
            if any(dt > policy["max_gap_ms"] and (onset is None or t <= onset[1] + policy["hold_ms"])
                   for dt,t in zip(gaps,times)):
                onset = None
            if any(dt > policy["max_gap_ms"] for dt in gaps):
                reasons.append("RECEIVE_GAP")
                settle = None
        if terminal is None or not ns or ns[-1] - terminal < policy["post_ms"]*1e6:
            reasons.append("SHORT_TAIL")
            settle = None
        if settle and (terminal is None or ns[-1] - max(terminal, settle[1] * 1e6) < policy["hold_ms"] * 1e6):
            settle = None
        if not evidence.get("tail_qualified", {}).get(side):
            reasons.append("UNQUALIFIED_TAIL")
            settle = None
        if onset is None or evidence.get("outcome") not in {"SETTLED", "TIMEOUT"}:
            settle = None
        if onset is None:
            reasons.append("START_UNQUALIFIED")
        if settle is None:
            reasons.append("SETTLE_UNQUALIFIED")
        latency = {}
        for label, a, b in (("receive_to_delivery_ms","received_monotonic_ns","recorder_delivery_ns"),
                            ("delivery_to_append_ms","recorder_delivery_ns","recorder_append_ns")):
            latency[label] = summarize([(p[b]-p[a])/1e6 for p in valid
                                        if type(p.get(b)) is int and type(p.get(a)) is int])
        result["arms"][side] = dict(start_host_ms=onset, stop_host_ms=settle, reasons=reasons,
                                    stationary_max_deviation_deg=max(noise, default=None),
                                    stationary_max_speed_deg_s=max(rates, default=None),
                                    receive_gap_ms=summarize(gaps), sequence_resets=resets,
                                    sequence_jumps=jumps, **latency)
    for event in ("start", "stop"):
        bounds = difference(*(result["arms"][s][event+"_host_ms"] for s in ("left", "right")))
        result[event+"_R_minus_L_host_ms"] = bounds
        result[event+"_classification"] = classify(bounds)
    return result


def write_samples(raw, path):
    packets = raw["endpoint_evidence"]["packets"] if "endpoint_evidence" in raw else raw.get("actual_packets", {})
    with path.open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["side", "received_monotonic_ns", "packet_sequence_index",
                         *[f"actual_joint_{i}_rad" for i in range(1, 7)]])
        for side, rows in packets.items():
            for row in rows:
                writer.writerow([side, row.get("received_monotonic_ns"), row.get("packet_sequence_index"),
                                 *row["actual_joints_rad"]])


def plot_samples(raw, path, thresholds):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    evidence = raw.get("endpoint_evidence", {})
    packets = evidence["packets"] if evidence else raw.get("actual_packets", {})
    origin = evidence.get("bind_observed_ns")
    if origin is None:
        origin = min((p["received_monotonic_ns"] for rows in packets.values() for p in rows
                      if type(p.get("received_monotonic_ns")) is int), default=0)
    report = analyze(raw, **thresholds)
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6), constrained_layout=True)
    for side, ax in zip(("left", "right"), axes):
        rows = [p for p in packets.get(side, []) if type(p.get("received_monotonic_ns")) is int]
        x = [(p["received_monotonic_ns"] - origin)/1e6 for p in rows]
        for joint in range(6):
            ax.plot(x, [degrees(p["actual_joints_rad"][joint]) for p in rows], ".",
                    markersize=2, label=f"J{joint+1}")
        for event, color in (("start", "green"), ("stop", "orange")):
            bounds = report["arms"][side][event + "_host_ms"]
            if bounds:
                ax.axvspan(bounds[0] - origin/1e6, bounds[1] - origin/1e6,
                           alpha=.22, color=color, label=event)
        ax.set_ylabel(f"{side.capitalize()} (deg)")
        ax.grid(alpha=.25)
    axes[0].legend(ncol=6)
    axes[-1].set_xlabel(f"Host receive time (ms from {origin} ns)")
    with path.open("xb") as output:
        fig.savefig(output, format="png")
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("--angle-deg", type=float)
    parser.add_argument("--speed-deg-s", type=float)
    parser.add_argument("--hold-ms", type=float)
    parser.add_argument("--max-gap-ms", type=float)
    parser.add_argument("--samples-csv", type=Path)
    parser.add_argument("--plot-png", type=Path)
    args = parser.parse_args()
    raw = json.loads(args.raw.read_bytes())
    if args.plot_png:
        plot_samples(raw, args.plot_png, dict(angle=args.angle_deg, speed=args.speed_deg_s,
                                              hold=args.hold_ms, max_gap=args.max_gap_ms))
    if args.samples_csv:
        write_samples(raw, args.samples_csv)
    print(json.dumps(analyze(raw, args.angle_deg,
                             args.speed_deg_s, args.hold_ms, args.max_gap_ms), indent=2))
