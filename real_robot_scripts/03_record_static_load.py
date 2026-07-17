#!/usr/bin/env python3
import argparse
import csv
import json
import re
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def safe_name(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    return value.strip("_") or "static_load"


class StaticLoadLogger(Node):
    def __init__(self, output_dir: Path) -> None:
        super().__init__("dual_arm_static_load_logger")
        self.output_dir = output_dir
        self.started_monotonic = time.monotonic()
        self.latest_joint: Dict[str, List[float]] = {"left": [], "right": []}
        self.sample_counts: Dict[str, int] = {"left": 0, "right": 0}

        self.csv_path = output_dir / "static_load_samples.csv"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)

        header = ["elapsed_sec", "side"]
        header += [f"current_j{i}" for i in range(1, 7)]
        header += [f"torque_j{i}" for i in range(1, 7)]
        header += [f"monitor_velocity_j{i}" for i in range(1, 7)]
        header += [f"joint_position_j{i}" for i in range(1, 7)]
        self.writer.writerow(header)
        self.csv_file.flush()

        self.create_subscription(
            JointState,
            "/left_jaka_driver/joint_position",
            lambda msg: self.on_joint("left", msg),
            20,
        )
        self.create_subscription(
            JointState,
            "/right_jaka_driver/joint_position",
            lambda msg: self.on_joint("right", msg),
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            "/left_jaka_driver/joint_monitor_raw",
            lambda msg: self.on_monitor("left", msg),
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            "/right_jaka_driver/joint_monitor_raw",
            lambda msg: self.on_monitor("right", msg),
            20,
        )

    def on_joint(self, side: str, msg: JointState) -> None:
        self.latest_joint[side] = list(msg.position[:6])

    def on_monitor(self, side: str, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 18:
            self.get_logger().warning(
                f"{side} joint_monitor_raw has {len(data)} values; expected at least 18"
            )
            return

        current = data[0:6]
        torque = data[6:12]
        velocity = data[12:18]
        joint = self.latest_joint[side]
        if len(joint) < 6:
            joint = [float("nan")] * 6

        elapsed = time.monotonic() - self.started_monotonic
        self.writer.writerow(
            [f"{elapsed:.9f}", side]
            + [f"{x:.12g}" for x in current]
            + [f"{x:.12g}" for x in torque]
            + [f"{x:.12g}" for x in velocity]
            + [f"{x:.12g}" for x in joint[:6]]
        )
        self.csv_file.flush()
        self.sample_counts[side] += 1

    def close(self) -> None:
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only static dual-arm current/torque logger."
    )
    parser.add_argument(
        "--name",
        default="static_load",
        help="Experiment name used in the output folder.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="Recording duration in seconds. Use 0 to run until Ctrl+C.",
    )
    parser.add_argument(
        "--output-root",
        default=str(
            Path.home()
            / "jaka_ws"
            / "dual_arm_app"
            / "experiment_logs"
        ),
        help="Root directory for experiment logs.",
    )
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_root).expanduser() / (
        f"{timestamp}_{safe_name(args.name)}"
    )
    output_dir.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    node = StaticLoadLogger(output_dir)
    started_at = datetime.now().astimezone().isoformat()

    print(f"Recording to: {output_dir}")
    print("Read-only logger: no robot command is sent.")
    if args.duration > 0:
        print(f"Duration: {args.duration:.1f} s")
    else:
        print("Duration: until Ctrl+C")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0:
                elapsed = time.monotonic() - node.started_monotonic
                if elapsed >= args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        duration_sec = time.monotonic() - node.started_monotonic
        node.close()

        metadata = {
            "status": "completed",
            "experiment_directory": str(output_dir),
            "started_at": started_at,
            "finished_at": datetime.now().astimezone().isoformat(),
            "duration_sec": duration_sec,
            "sample_counts": node.sample_counts,
            "topics": {
                "left_monitor": "/left_jaka_driver/joint_monitor_raw",
                "right_monitor": "/right_jaka_driver/joint_monitor_raw",
                "left_joint": "/left_jaka_driver/joint_position",
                "right_joint": "/right_jaka_driver/joint_position",
            },
            "monitor_layout": {
                "0_to_5": "current J1-J6",
                "6_to_11": "SDK-reported torque J1-J6",
                "12_to_17": "SDK-reported velocity J1-J6",
            },
            "notes": [
                "This logger is read-only and sends no robot commands.",
                "SDK-reported torque is not yet treated as calibrated N*m.",
                "Use the same robot pose and tool configuration for unloaded and loaded tests.",
            ],
        }
        (output_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

        node.destroy_node()
        rclpy.shutdown()

    print(f"Saved: {output_dir / 'static_load_samples.csv'}")
    print(f"Left samples : {node.sample_counts['left']}")
    print(f"Right samples: {node.sample_counts['right']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
