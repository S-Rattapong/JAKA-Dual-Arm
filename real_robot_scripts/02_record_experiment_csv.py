#!/usr/bin/env python3

import argparse
import csv
import json
import re
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg


class DualArmExperimentLogger(Node):
    def __init__(self, experiment_name: str, api_url: str):
        super().__init__("dual_arm_experiment_logger")

        self.api_url = api_url.rstrip("/")
        self.start_monotonic_ns = time.monotonic_ns()
        self.start_wall = datetime.now().astimezone()

        safe_name = re.sub(r"[^A-Za-z0-9_-]+", "_", experiment_name.strip())
        safe_name = safe_name.strip("_") or "experiment"

        timestamp = self.start_wall.strftime("%Y%m%d_%H%M%S")
        self.output_dir = (
            Path.home()
            / "jaka_ws"
            / "dual_arm_app"
            / "experiment_logs"
            / f"{timestamp}_{safe_name}"
        )
        self.output_dir.mkdir(parents=True, exist_ok=False)

        self.joint_path = self.output_dir / "joint_samples.csv"
        self.state_path = self.output_dir / "robot_state_samples.csv"
        self.metadata_path = self.output_dir / "metadata.json"

        self.joint_file = self.joint_path.open(
            "w", newline="", encoding="utf-8"
        )
        self.state_file = self.state_path.open(
            "w", newline="", encoding="utf-8"
        )

        self.joint_writer = csv.DictWriter(
            self.joint_file,
            fieldnames=[
                "receive_wall_iso",
                "receive_monotonic_ns",
                "elapsed_sec",
                "ros_stamp_ns",
                "side",
                "cycle",
                "completed_cycles",
                "step_index",
                "step_name",
                "sequence_status",
                "j1_rad",
                "j2_rad",
                "j3_rad",
                "j4_rad",
                "j5_rad",
                "j6_rad",
            ],
        )
        self.joint_writer.writeheader()

        self.state_writer = csv.DictWriter(
            self.state_file,
            fieldnames=[
                "receive_wall_iso",
                "receive_monotonic_ns",
                "elapsed_sec",
                "side",
                "cycle",
                "completed_cycles",
                "step_index",
                "step_name",
                "sequence_status",
                "motion_state",
                "power_state",
                "servo_state",
                "collision_state",
            ],
        )
        self.state_writer.writeheader()

        self.latest_sequence: Dict[str, Any] = {}
        self.joint_sample_count = 0
        self.state_sample_count = 0
        self.api_error_count = 0
        self.closed = False

        self.create_subscription(
            JointState,
            "/left_jaka_driver/joint_position",
            lambda msg: self.joint_callback("left", msg),
            50,
        )
        self.create_subscription(
            JointState,
            "/right_jaka_driver/joint_position",
            lambda msg: self.joint_callback("right", msg),
            50,
        )
        self.create_subscription(
            RobotMsg,
            "/left_jaka_driver/robot_states",
            lambda msg: self.state_callback("left", msg),
            50,
        )
        self.create_subscription(
            RobotMsg,
            "/right_jaka_driver/robot_states",
            lambda msg: self.state_callback("right", msg),
            50,
        )

        # Poll เฉพาะข้อมูล Cycle/Waypoint จาก Backend
        # ไม่ได้ส่งคำสั่งเคลื่อนที่
        self.create_timer(0.2, self.update_sequence_context)

        self.write_metadata(status="recording")

        print()
        print("==============================================")
        print("Dual-arm experiment recording started")
        print(f"Output directory: {self.output_dir}")
        print("Press Ctrl+C to stop and save metadata.")
        print("==============================================")
        print()

    def elapsed_sec(self, receive_ns: int) -> float:
        return (receive_ns - self.start_monotonic_ns) / 1_000_000_000.0

    @staticmethod
    def wall_iso() -> str:
        return datetime.now().astimezone().isoformat(timespec="milliseconds")

    def sequence_fields(self) -> Dict[str, Any]:
        seq = self.latest_sequence or {}
        return {
            "cycle": seq.get("current_cycle", ""),
            "completed_cycles": seq.get("completed_cycles", ""),
            "step_index": seq.get("current_index", ""),
            "step_name": seq.get("current_name", ""),
            "sequence_status": seq.get("status", ""),
        }

    def update_sequence_context(self) -> None:
        try:
            with urllib.request.urlopen(
                f"{self.api_url}/api/status",
                timeout=0.15,
            ) as response:
                payload = json.load(response)

            active = payload.get("active_sequence")
            self.latest_sequence = active if isinstance(active, dict) else {}

        except Exception:
            # Logger ยังทำงานต่อได้ แม้ Backend ปิดหรือ API ตอบไม่ทัน
            self.api_error_count += 1

    def joint_callback(self, side: str, msg: JointState) -> None:
        positions = list(msg.position)[:6]
        if len(positions) != 6:
            return

        receive_ns = time.monotonic_ns()
        ros_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

        row = {
            "receive_wall_iso": self.wall_iso(),
            "receive_monotonic_ns": receive_ns,
            "elapsed_sec": f"{self.elapsed_sec(receive_ns):.9f}",
            "ros_stamp_ns": ros_stamp_ns,
            "side": side,
            **self.sequence_fields(),
            "j1_rad": positions[0],
            "j2_rad": positions[1],
            "j3_rad": positions[2],
            "j4_rad": positions[3],
            "j5_rad": positions[4],
            "j6_rad": positions[5],
        }

        self.joint_writer.writerow(row)
        self.joint_sample_count += 1

        if self.joint_sample_count % 20 == 0:
            self.joint_file.flush()

    def state_callback(self, side: str, msg: RobotMsg) -> None:
        receive_ns = time.monotonic_ns()

        row = {
            "receive_wall_iso": self.wall_iso(),
            "receive_monotonic_ns": receive_ns,
            "elapsed_sec": f"{self.elapsed_sec(receive_ns):.9f}",
            "side": side,
            **self.sequence_fields(),
            "motion_state": int(msg.motion_state),
            "power_state": int(msg.power_state),
            "servo_state": int(msg.servo_state),
            "collision_state": int(msg.collision_state),
        }

        self.state_writer.writerow(row)
        self.state_sample_count += 1

        if self.state_sample_count % 20 == 0:
            self.state_file.flush()

    def write_metadata(self, status: str) -> None:
        now = datetime.now().astimezone()
        duration_sec = (
            time.monotonic_ns() - self.start_monotonic_ns
        ) / 1_000_000_000.0

        metadata = {
            "status": status,
            "experiment_directory": str(self.output_dir),
            "started_at": self.start_wall.isoformat(timespec="seconds"),
            "updated_at": now.isoformat(timespec="seconds"),
            "duration_sec": duration_sec,
            "joint_sample_count": self.joint_sample_count,
            "robot_state_sample_count": self.state_sample_count,
            "api_status_poll_error_count": self.api_error_count,
            "api_url": self.api_url,
            "topics": {
                "left_joint": "/left_jaka_driver/joint_position",
                "right_joint": "/right_jaka_driver/joint_position",
                "left_state": "/left_jaka_driver/robot_states",
                "right_state": "/right_jaka_driver/robot_states",
            },
            "signal_limitations": {
                "joint_velocity_available": False,
                "joint_effort_available": False,
                "force_torque_available": False,
                "nominal_feedback_rate_hz": "approximately 8.5–8.8",
            },
        }

        self.metadata_path.write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def close_logger(self) -> None:
        if self.closed:
            return

        self.closed = True

        self.joint_file.flush()
        self.state_file.flush()

        self.write_metadata(status="completed")

        self.joint_file.close()
        self.state_file.close()

        print()
        print("==============================================")
        print("Experiment recording stopped")
        print(f"Joint samples : {self.joint_sample_count}")
        print(f"State samples : {self.state_sample_count}")
        print(f"Saved at      : {self.output_dir}")
        print("==============================================")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read-only dual JAKA experiment CSV logger"
    )
    parser.add_argument(
        "--name",
        default="dual_arm_test",
        help="Experiment name used in the output directory",
    )
    parser.add_argument(
        "--api-url",
        default="http://127.0.0.1:8000",
        help="Backend URL used only to read cycle/waypoint context",
    )
    args = parser.parse_args()

    rclpy.init()
    node = DualArmExperimentLogger(args.name, args.api_url)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_logger()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
