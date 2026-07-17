#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

JOINT_COLS = [f"j{i}_rad" for i in range(1, 7)]


def safe_float(v):
    try:
        if pd.isna(v):
            return None
        return float(v)
    except Exception:
        return None


def measured_rate(df, side):
    x = df[df["side"] == side].copy()
    if len(x) < 2:
        return None
    if "ros_stamp_ns" in x.columns:
        t = pd.to_numeric(x["ros_stamp_ns"], errors="coerce").dropna().to_numpy(dtype=float) / 1e9
    else:
        t = pd.to_numeric(x["elapsed_sec"], errors="coerce").dropna().to_numpy(dtype=float)
    if len(t) < 2:
        return None
    dt = np.diff(np.sort(t))
    dt = dt[(dt > 0) & np.isfinite(dt)]
    if len(dt) == 0:
        return None
    return float(1.0 / np.median(dt))


def step_order(states):
    x = states.dropna(subset=["step_index", "step_name"]).copy()
    if x.empty:
        return []
    x["step_index"] = pd.to_numeric(x["step_index"], errors="coerce")
    x = x.dropna(subset=["step_index"])
    x["step_index"] = x["step_index"].astype(int)
    pairs = (
        x.sort_values("elapsed_sec")
         .drop_duplicates(["step_index"], keep="first")[["step_index", "step_name"]]
    )
    return [(int(r.step_index), str(r.step_name)) for r in pairs.itertuples(index=False)]


def motion_window(states, step_idx, side):
    x = states[
        (states["side"] == side)
        & (pd.to_numeric(states["step_index"], errors="coerce") == step_idx)
    ].copy()
    if x.empty:
        return None

    x["elapsed_sec"] = pd.to_numeric(x["elapsed_sec"], errors="coerce")
    x["motion_state"] = pd.to_numeric(x["motion_state"], errors="coerce")
    x = x.dropna(subset=["elapsed_sec", "motion_state"]).sort_values("elapsed_sec")
    if x.empty:
        return None

    running = x[
        (x["motion_state"] != 0)
        & (x["sequence_status"].astype(str) == "running")
    ]
    if running.empty:
        running = x[x["motion_state"] != 0]
    if running.empty:
        return None

    start = float(running["elapsed_sec"].iloc[0])

    stopped_after_start = x[
        (x["elapsed_sec"] > start)
        & (x["motion_state"] == 0)
    ]
    if not stopped_after_start.empty:
        stop = float(stopped_after_start["elapsed_sec"].iloc[0])
    else:
        stop = float(running["elapsed_sec"].iloc[-1])

    if stop <= start:
        return None
    return start, stop


def extract_joint_segment(joints, side, t0, t1, pad=0.12):
    x = joints[
        (joints["side"] == side)
        & (pd.to_numeric(joints["elapsed_sec"], errors="coerce") >= t0 - pad)
        & (pd.to_numeric(joints["elapsed_sec"], errors="coerce") <= t1 + pad)
    ].copy()
    if x.empty:
        return x
    x["elapsed_sec"] = pd.to_numeric(x["elapsed_sec"], errors="coerce")
    for c in JOINT_COLS:
        x[c] = pd.to_numeric(x[c], errors="coerce")
    return x.dropna(subset=["elapsed_sec", *JOINT_COLS]).sort_values("elapsed_sec")


def progress_series(seg, t0, t1):
    if len(seg) < 2:
        return None, None
    t = seg["elapsed_sec"].to_numpy(dtype=float)
    q = seg[JOINT_COLS].to_numpy(dtype=float)

    start_candidates = np.where(t <= t0 + 0.12)[0]
    end_candidates = np.where(t >= t1 - 0.12)[0]
    i0 = int(start_candidates[-1]) if len(start_candidates) else 0
    i1 = int(end_candidates[0]) if len(end_candidates) else len(t) - 1
    if i1 <= i0:
        i0, i1 = 0, len(t) - 1

    q0 = q[i0]
    q1 = q[i1]
    d = q1 - q0
    denom = float(np.dot(d, d))
    if denom < 1e-12:
        return None, None

    p = ((q - q0) @ d) / denom
    p = np.clip(p, 0.0, 1.0)
    return t, p


def analyze_step(joints, states, step_idx, step_name):
    lw = motion_window(states, step_idx, "left")
    rw = motion_window(states, step_idx, "right")
    row = {
        "step_index": step_idx,
        "step_name": step_name,
        "left_start_sec": None,
        "right_start_sec": None,
        "start_difference_sec": None,
        "left_stop_sec": None,
        "right_stop_sec": None,
        "stop_difference_sec": None,
        "left_duration_sec": None,
        "right_duration_sec": None,
        "duration_difference_sec": None,
        "max_progress_mismatch_percent": None,
        "rms_progress_mismatch_percent": None,
        "result": "missing_motion_feedback",
    }
    progress_payload = None
    if lw is None or rw is None:
        return row, progress_payload

    ls, le = lw
    rs, re = rw
    row.update({
        "left_start_sec": ls,
        "right_start_sec": rs,
        "start_difference_sec": abs(ls - rs),
        "left_stop_sec": le,
        "right_stop_sec": re,
        "stop_difference_sec": abs(le - re),
        "left_duration_sec": le - ls,
        "right_duration_sec": re - rs,
        "duration_difference_sec": abs((le - ls) - (re - rs)),
    })

    lseg = extract_joint_segment(joints, "left", ls, le)
    rseg = extract_joint_segment(joints, "right", rs, re)
    lt, lp = progress_series(lseg, ls, le)
    rt, rp = progress_series(rseg, rs, re)

    if lt is not None and rt is not None:
        overlap_start = max(float(lt.min()), float(rt.min()), ls, rs)
        overlap_end = min(float(lt.max()), float(rt.max()), le, re)
        if overlap_end > overlap_start:
            grid = np.linspace(overlap_start, overlap_end, 200)
            lpi = np.interp(grid, lt, lp)
            rpi = np.interp(grid, rt, rp)
            mismatch = np.abs(lpi - rpi) * 100.0
            row["max_progress_mismatch_percent"] = float(np.max(mismatch))
            row["rms_progress_mismatch_percent"] = float(np.sqrt(np.mean(mismatch ** 2)))
            progress_payload = {
                "time": grid,
                "left_progress": lpi * 100.0,
                "right_progress": rpi * 100.0,
                "mismatch": mismatch,
            }

    row["result"] = "ok"
    return row, progress_payload


def main():
    parser = argparse.ArgumentParser(description="Analyze dual-arm JAKA CSV experiment log")
    parser.add_argument("experiment_dir", help="Directory containing joint_samples.csv, robot_state_samples.csv, metadata.json")
    args = parser.parse_args()

    exp_dir = Path(args.experiment_dir).expanduser().resolve()
    joint_path = exp_dir / "joint_samples.csv"
    state_path = exp_dir / "robot_state_samples.csv"
    metadata_path = exp_dir / "metadata.json"
    for p in (joint_path, state_path, metadata_path):
        if not p.exists():
            raise SystemExit(f"Missing required file: {p}")

    joints = pd.read_csv(joint_path)
    states = pd.read_csv(state_path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

    required_joint = {"elapsed_sec", "side", *JOINT_COLS}
    required_state = {"elapsed_sec", "side", "motion_state", "step_index", "step_name"}
    missing_joint = sorted(required_joint - set(joints.columns))
    missing_state = sorted(required_state - set(states.columns))
    if missing_joint:
        raise SystemExit(f"joint_samples.csv missing columns: {missing_joint}")
    if missing_state:
        raise SystemExit(f"robot_state_samples.csv missing columns: {missing_state}")

    joints["elapsed_sec"] = pd.to_numeric(joints["elapsed_sec"], errors="coerce")
    states["elapsed_sec"] = pd.to_numeric(states["elapsed_sec"], errors="coerce")
    states["motion_state"] = pd.to_numeric(states["motion_state"], errors="coerce")

    out_dir = exp_dir / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    ordered_steps = step_order(states)
    rows = []
    progress_by_step = {}
    for idx, name in ordered_steps:
        row, payload = analyze_step(joints, states, idx, name)
        rows.append(row)
        if payload is not None:
            progress_by_step[idx] = payload

    step_df = pd.DataFrame(rows)
    step_df.to_csv(out_dir / "step_summary.csv", index=False)

    left_rate = measured_rate(joints, "left")
    right_rate = measured_rate(joints, "right")

    running = states[
        states["sequence_status"].astype(str).isin(["running", "delay", "done"])
        & states["step_index"].notna()
    ] if "sequence_status" in states.columns else states[states["step_index"].notna()]

    program_start = safe_float(running["elapsed_sec"].min()) if not running.empty else None
    done_rows = states[states.get("sequence_status", pd.Series(index=states.index, dtype=object)).astype(str) == "done"]
    program_end = safe_float(done_rows["elapsed_sec"].max()) if not done_rows.empty else safe_float(running["elapsed_sec"].max()) if not running.empty else None

    collision_count = int((states["collision_state"] != 0).sum()) if "collision_state" in states.columns else None
    servo_drop_count = int((states["servo_state"] != 1).sum()) if "servo_state" in states.columns else None
    power_drop_count = int((states["power_state"] != 1).sum()) if "power_state" in states.columns else None

    ok_steps = step_df[step_df["result"] == "ok"] if not step_df.empty else step_df
    summary = {
        "experiment_directory": str(exp_dir),
        "analysis_directory": str(out_dir),
        "log_duration_sec": safe_float(metadata.get("duration_sec")),
        "program_start_sec": program_start,
        "program_end_sec": program_end,
        "program_cycle_time_sec": (program_end - program_start) if program_start is not None and program_end is not None else None,
        "joint_samples": int(len(joints)),
        "robot_state_samples": int(len(states)),
        "measured_joint_rate_hz": {
            "left": left_rate,
            "right": right_rate,
        },
        "step_count": int(len(step_df)),
        "successful_step_analyses": int(len(ok_steps)),
        "collision_sample_count": collision_count,
        "servo_not_ready_sample_count": servo_drop_count,
        "power_off_sample_count": power_drop_count,
        "timing_summary_sec": {
            "median_start_difference": safe_float(ok_steps["start_difference_sec"].median()) if not ok_steps.empty else None,
            "max_start_difference": safe_float(ok_steps["start_difference_sec"].max()) if not ok_steps.empty else None,
            "median_stop_difference": safe_float(ok_steps["stop_difference_sec"].median()) if not ok_steps.empty else None,
            "max_stop_difference": safe_float(ok_steps["stop_difference_sec"].max()) if not ok_steps.empty else None,
            "median_duration_difference": safe_float(ok_steps["duration_difference_sec"].median()) if not ok_steps.empty else None,
            "max_duration_difference": safe_float(ok_steps["duration_difference_sec"].max()) if not ok_steps.empty else None,
        },
        "progress_summary_percent": {
            "median_max_mismatch": safe_float(ok_steps["max_progress_mismatch_percent"].median()) if not ok_steps.empty else None,
            "worst_max_mismatch": safe_float(ok_steps["max_progress_mismatch_percent"].max()) if not ok_steps.empty else None,
            "median_rms_mismatch": safe_float(ok_steps["rms_progress_mismatch_percent"].median()) if not ok_steps.empty else None,
            "worst_rms_mismatch": safe_float(ok_steps["rms_progress_mismatch_percent"].max()) if not ok_steps.empty else None,
        },
        "limitations": [
            "motion timing resolution is limited by approximately 15 Hz feedback",
            "joint velocity and effort are not provided by the current driver",
            "load sharing cannot be calculated from this dataset",
            "progress is estimated from joint-space projection and is not Cartesian TCP progress",
        ],
    }
    (out_dir / "experiment_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    # Plot 1: motion-state timeline
    plt.figure(figsize=(12, 4))
    for side, offset in (("left", 0.05), ("right", -0.05)):
        x = states[states["side"] == side].sort_values("elapsed_sec")
        y = (x["motion_state"].to_numpy(dtype=float) != 0).astype(float) + offset
        plt.step(x["elapsed_sec"], y, where="post", label=side.capitalize())
    plt.yticks([0, 1], ["Stopped", "Moving"])
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Motion state")
    plt.title("Dual-arm Motion State Timeline")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "motion_state_timeline.png", dpi=180)
    plt.close()

    # Plot 2: normalized progress for all analyzed steps
    plt.figure(figsize=(12, 6))
    for idx, payload in progress_by_step.items():
        name = next((n for i, n in ordered_steps if i == idx), str(idx))
        t = payload["time"]
        plt.plot(t, payload["left_progress"], linewidth=1.0, label=f"{idx}:{name} L")
        plt.plot(t, payload["right_progress"], linewidth=1.0, linestyle="--", label=f"{idx}:{name} R")
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Normalized joint-space progress (%)")
    plt.title("Dual-arm Normalized Motion Progress")
    plt.grid(True, alpha=0.3)
    if len(progress_by_step) <= 8:
        plt.legend(ncol=2, fontsize=8)
    plt.tight_layout()
    plt.savefig(out_dir / "normalized_progress.png", dpi=180)
    plt.close()

    # Plot 3: mismatch per step
    if not step_df.empty:
        plot_df = step_df.dropna(subset=["max_progress_mismatch_percent"]).copy()
        if not plot_df.empty:
            labels = [f"{int(i)}:{n}" for i, n in zip(plot_df["step_index"], plot_df["step_name"])]
            x = np.arange(len(plot_df))
            plt.figure(figsize=(13, 5))
            plt.bar(x, plot_df["max_progress_mismatch_percent"].to_numpy(dtype=float))
            plt.xticks(x, labels, rotation=60, ha="right")
            plt.ylabel("Maximum progress mismatch (%)")
            plt.xlabel("Program step")
            plt.title("Maximum Dual-arm Progress Mismatch per Step")
            plt.grid(True, axis="y", alpha=0.3)
            plt.tight_layout()
            plt.savefig(out_dir / "progress_mismatch_per_step.png", dpi=180)
            plt.close()

    # Plot 4/5: joint trajectories
    for side in ("left", "right"):
        x = joints[joints["side"] == side].sort_values("elapsed_sec")
        plt.figure(figsize=(12, 6))
        for c in JOINT_COLS:
            plt.plot(x["elapsed_sec"], x[c], label=c.replace("_rad", "").upper())
        plt.xlabel("Elapsed time (s)")
        plt.ylabel("Joint position (rad)")
        plt.title(f"{side.capitalize()} Arm Joint Trajectory")
        plt.grid(True, alpha=0.3)
        plt.legend(ncol=3)
        plt.tight_layout()
        plt.savefig(out_dir / f"joint_trajectory_{side}.png", dpi=180)
        plt.close()

    print("Analysis completed")
    print(f"Experiment : {exp_dir}")
    print(f"Output     : {out_dir}")
    print(f"Steps      : {len(step_df)}")
    print(f"Joint rate : left={left_rate:.2f} Hz, right={right_rate:.2f} Hz")
    if summary["program_cycle_time_sec"] is not None:
        print(f"Cycle time : {summary['program_cycle_time_sec']:.3f} s")
    print()
    print(step_df[
        [
            "step_index", "step_name", "start_difference_sec",
            "stop_difference_sec", "max_progress_mismatch_percent",
            "rms_progress_mismatch_percent", "result"
        ]
    ].to_string(index=False))


if __name__ == "__main__":
    main()
