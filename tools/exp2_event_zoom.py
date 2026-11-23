"""Offline 2x2 zoom plots of host-observed L/R start and stop evidence.

Shows actual feedback points, maximal joint deviation from the planned
start/goal, and speed between host receive samples. Does not indicate
physical movement timestamps or external metrology.
"""
from __future__ import annotations

import argparse
import json
from math import degrees
from pathlib import Path

from tools.exp2_host_endpoints import analyze


def plot_zooms(raw, output, *, start_before_ms=1250, start_after_ms=2000,
               stop_before_ms=1500, stop_after_ms=2500):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    report = analyze(raw)
    evidence = raw.get("endpoint_evidence") or {}
    packets = evidence.get("packets", raw.get("actual_packets", {}))
    samples = raw["planned"]["samples"]
    origin_by_event = {
        "start": evidence.get("bind_observed_ns"),
        "stop": evidence.get("terminal_observed_ns"),
    }
    fig, axes = plt.subplots(2, 2, figsize=(15, 9), constrained_layout=True)
    for i, side in enumerate(("left", "right")):
        rows = [p for p in packets.get(side, []) if isinstance(p.get("received_monotonic_ns"), int)
                and isinstance(p.get("actual_joints_rad"), list) and len(p["actual_joints_rad"]) == 6]
        rows.sort(key=lambda p: p["received_monotonic_ns"])
        for j, event in enumerate(("start", "stop")):
            ax = axes[i, j]
            origin = origin_by_event[event]
            if origin is None:
                if not rows:
                    ax.text(.5, .5, "No host samples", ha="center", va="center", transform=ax.transAxes)
                    continue
                origin = rows[0]["received_monotonic_ns"] if event == "start" else rows[-1]["received_monotonic_ns"]
            reference = samples[0 if event == "start" else -1][side]["joints_rad"]
            before = start_before_ms if event == "start" else stop_before_ms
            after = start_after_ms if event == "start" else stop_after_ms
            in_window = [p for p in rows if -before <= (p["received_monotonic_ns"] - origin)/1e6 <= after]
            times = [(p["received_monotonic_ns"] - origin)/1e6 for p in in_window]
            deviation = [
                max(abs(degrees(a - b)) for a,b in zip(p["actual_joints_rad"], reference))
                for p in in_window
            ]
            ax.plot(times, deviation, "o-", markersize=4, linewidth=1.1,
                    label="Actual sampled max |joint - endpoint| (deg)")
            ax.axhline(report["policy"]["angle_deg"], color="#a32633", linestyle="--",
                       linewidth=1.4, label=f"Angle threshold: {report['policy']['angle_deg']} deg")
            event_bound = report["arms"][side][f"{event}_host_ms"]
            if event_bound:
                ax.axvspan(event_bound[0]-origin/1e6,
                           event_bound[1]-origin/1e6, color="seagreen", alpha=.22,
                           label=f"{event} event bracket")
            ax.axvline(0, color="#666", linestyle=":", linewidth=1)
            ax.set_xlim(-before, after)
            ax.set_xlabel("Host monotonic time relative to observed boundary (ms)")
            ax.set_ylabel("Max joint deviation (deg)")
            ax.grid(alpha=.22)
            if times:
                ax.set_ylim(bottom=0)
            status=report[f"{event}_classification"] if event in ("start","stop") else "N/A"
            ax.set_title(f"{side.upper()} / {event.upper()} — {len(in_window)} actual packets | {status}")
            if event == "stop" and len(in_window)>1:
                vx=[]; vt=[]
                for left,right in zip(in_window, in_window[1:]):
                    dt=(right["received_monotonic_ns"]-left["received_monotonic_ns"])/1e9
                    if dt<=0:
                        continue
                    vx.append(max(abs(degrees(a-b))/dt for a,b in zip(
                        right["actual_joints_rad"], left["actual_joints_rad"])))
                    vt.append((right["received_monotonic_ns"]-origin)/1e6)
                ax2=ax.twinx()
                ax2.plot(vt,vx,"x",color="#a16d16",markersize=4,alpha=.75,
                         label="Sample-to-sample max speed (deg/s)")
                ax2.axhline(report["policy"]["speed_deg_s"],color="#a16d16",
                            linestyle="--",alpha=.6)
                ax2.set_ylabel("Sample-to-sample max joint speed (deg/s)")
            reasons=", ".join(report["arms"][side]["reasons"][:3]) or "No data warnings"
            ax.text(.01,.97,reasons,transform=ax.transAxes,va="top",fontsize=8,
                    bbox=dict(facecolor="white",alpha=.8,edgecolor="none"))
    fig.suptitle(
        f"EXP2 start / settle host-feedback zoom — {raw.get('run_id','unknown')}\n"
        f"R-L start {report['start_R_minus_L_host_ms']} ms | "
        f"R-L settle {report['stop_R_minus_L_host_ms']} ms (not controller acquisition timestamps)",
        fontsize=11)
    with Path(output).open("xb") as stream:
        fig.savefig(stream, format="png", dpi=120)
    plt.close(fig)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("output_png", type=Path)
    args = parser.parse_args()
    raw = json.loads(args.raw.read_text(encoding="utf-8"))
    report = plot_zooms(raw, args.output_png)
    print(json.dumps({"output": str(args.output_png),
                      "start_R_minus_L_host_ms": report["start_R_minus_L_host_ms"],
                      "stop_R_minus_L_host_ms": report["stop_R_minus_L_host_ms"],
                      "start_ordering": report["start_classification"],
                      "stop_ordering": report["stop_classification"]}))
