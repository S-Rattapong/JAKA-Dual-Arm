#!/usr/bin/env python3
"""Fixed operator driver entrypoint. Invoking this file CAN power/enable a robot."""
import fcntl
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dual_arm_app.backend.operator_runtime import DRIVERS, driver_matches, scan_processes, user_bus_env


def ros_driver_present(side):
    # DDS graph discovery only; never call a robot service or create an SDK session.
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node(f"operator_{side}_duplicate_preflight_{os.getpid()}")
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        spec = DRIVERS[side]
        return (any(name == spec["node"] for name, _ in node.get_node_names_and_namespaces())
                or any(name.startswith(spec["prefix"] + "/") for name, _ in node.get_service_names_and_types()))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in DRIVERS:
        raise SystemExit("Expected fixed side left or right")
    side = sys.argv[1]
    os.environ.update(user_bus_env())
    lock_dir = Path(os.environ["XDG_RUNTIME_DIR"]) / "jaka-operator"
    lock_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    lock = (lock_dir / f"{side}-start.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(f"{side}: operator driver start already owned; no second SDK session", flush=True)
        return
    def existing():
        return any(driver_matches(p["argv"], side) for p in scan_processes())
    if existing() or ros_driver_present(side) or existing():
        print(f"{side}: existing native/wrapper/ROS driver detected; no second SDK session", flush=True)
        return
    spec = DRIVERS[side]
    os.set_inheritable(lock.fileno(), True)
    print(f"{side}: starting driver, connect + power + enable (auto_enable=true)", flush=True)
    os.execvp("ros2", ["ros2", "run", "jaka_driver", "jaka_driver", "--ros-args",
                        "-r", f"__node:={spec['node']}", "-p", f"ip:={spec['ip']}",
                        "-p", "read_only:=false", "-p", "auto_enable:=true",
                        "-p", f"service_prefix:={spec['prefix']}"])


if __name__ == "__main__":
    main()
