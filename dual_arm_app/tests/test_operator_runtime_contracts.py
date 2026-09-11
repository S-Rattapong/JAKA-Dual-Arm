from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def head(path):
    return subprocess.check_output(["git", "show", f"HEAD:{path}"], cwd=ROOT, text=True)


def cpp_function(source, marker):
    start = source.index(marker)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated function {marker}")


def test_dev_driver_scripts_are_byte_identical_to_head():
    for path in ("real_robot_scripts/02_left_driver.sh", "real_robot_scripts/03_right_driver.sh"):
        assert (ROOT / path).read_bytes() == subprocess.check_output(["git", "show", f"HEAD:{path}"], cwd=ROOT)


def test_operator_driver_entries_preserve_fixed_identity_and_auto_enable_contract():
    left = read("operator_runtime/02_left_operator_driver.sh")
    right = read("operator_runtime/03_right_operator_driver.sh")
    guard = read("operator_runtime/driver_guard.py")
    for source, side, ip in ((left, "left", "192.168.0.1"), (right, "right", "192.168.0.2")):
        assert f"{side}_jaka_driver_node" in source
        assert ip in source
        assert "read_only:=false" in source
        assert "auto_enable:=true" in source
        assert f"service_prefix:=/{side}_jaka_driver" in source
        assert f"driver_guard.py\" {side}" in source
    for literal in (
        '"read_only:=false"', '"auto_enable:=true"',
        'f"ip:={spec[\'ip\']}"', 'f"__node:={spec[\'node\']}"',
        'f"service_prefix:={spec[\'prefix\']}"',
    ):
        assert literal in guard
    assert 'os.execvp("ros2"' in guard


def test_operator_moveit_and_web_commands_are_exact_and_localhost_only():
    moveit = read("operator_runtime/04_operator_moveit.sh")
    command = "exec ros2 launch jaka_a12_moveit_config dual_moveit_rviz.launch.py use_rviz:=false use_joint_state_publisher_gui:=false"
    assert command in moveit
    web = read("operator_runtime/06_operator_web_backend.sh")
    assert "export JAKA_OPERATOR_MODE=1" in web
    assert "--host 127.0.0.1" in web
    assert "--port 8000" in web
    assert "--host 0.0.0.0" not in web


def test_launcher_shutdown_stops_only_operator_web_and_moveit_and_never_sends_stop():
    launcher = read("operator_runtime/launch_operator_hmi.py")
    shutdown = launcher.split("def shutdown_when_safe", 1)[1].split("def main", 1)[0]
    assert 'systemd.command("stop", "web")' in shutdown
    assert 'systemd.command("stop", "moveit")' in shutdown
    assert 'systemd.command("stop", "left")' not in shutdown
    assert 'systemd.command("stop", "right")' not in shutdown
    assert "/api/stop" not in shutdown
    assert "stop_move" not in shutdown
    assert "request_stop" not in shutdown
    assert "motion_abort" not in shutdown
    assert 'status.get("shutdown", {}).get("safe") is True' in launcher


def test_installer_only_installs_and_reloads_user_units_without_starting_services():
    installer = read("operator_runtime/install_operator_services.sh")
    assert "systemctl --user daemon-reload" in installer
    assert "systemctl --user start" not in installer
    assert "systemctl --user restart" not in installer
    assert "systemctl --user enable" not in installer
    assert "sudo" not in installer


def test_desktop_entry_has_no_placeholder_and_uses_terminal_launcher_with_home_expansion():
    desktop = read("operator_runtime/JAKA Dual Arm Control.desktop")
    assert "@LAUNCHER@" not in desktop
    assert "Name=JAKA Dual Arm Control" in desktop
    assert "Exec=gnome-terminal -- /bin/bash -lc" in desktop
    assert "exec ~/jaka_ws/operator_runtime/launch_operator_hmi.sh" in desktop
    assert not (ROOT / "operator_runtime/JAKA-Operator.desktop").exists()


def test_backend_has_one_operator_owner_client_map_and_router_registration():
    backend = read("dual_arm_app/backend/dual_jaka_web_backend.py")
    assert backend.count("from std_srvs.srv import Empty, SetBool") == 1
    assert backend.count("self.operator_runtime = OperatorRuntime()") == 1
    assert backend.count("self.operator_state_clients = {") == 1
    assert backend.count("app.include_router(system_control_router(node))") == 1
    assert "install_operator_api" not in backend
    assert 'status["shutdown"] = shutdown_decision' in backend
    assert "passive_execution_snapshot()" in backend


def test_phase5_passive_snapshot_is_copy_only_and_operator_status_does_not_call_is_active():
    coordinator = read("dual_arm_app/backend/phase5_execution_coordinator.py")
    passive = coordinator.split("def passive_execution_snapshot", 1)[1].split("def is_active", 1)[0]
    assert "with self._lock" in passive
    assert "return dict(self._execution)" in passive
    assert "_read_execution_feedback" not in passive
    assert "_abort_callback" not in passive

    backend = read("dual_arm_app/backend/dual_jaka_web_backend.py")
    operator_status = backend.split("def operator_status", 1)[1].split("def operator_connect", 1)[0]
    assert "passive_execution_snapshot" in operator_status
    assert ".is_active()" not in operator_status
    assert "request_stop" not in operator_status


def test_driver_robotmsg_callback_is_unchanged_and_state_control_uses_single_session_safety_locks():
    path = "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp"
    current = read(path)
    baseline = head(path)
    assert cpp_function(current, "void robot_states_callback") == cpp_function(baseline, "void robot_states_callback")

    control = cpp_function(current, "void operator_state_control")
    assert "JAKAZuRobot" not in control
    assert "login_in" not in control and "login_out" not in control
    assert "phase5_sdk_control_window_active" in control
    assert "phase5_trajectory_active" in control
    assert "phase5_trajectory_mutex" in control
    assert "phase5_telemetry_gate_mutex" in control
    assert control.index("phase5_trajectory_mutex") < control.index("phase5_telemetry_gate_mutex")
    assert "robot.get_robot_status_simple" in control
    assert "robot.is_in_pos" in control
    assert "robot.get_program_state" in control
    assert "robot.is_in_drag_mode" in control
    assert "robot.power_on()" in control and "robot.power_off()" in control
    assert "robot.enable_robot()" in control and "robot.disable_robot()" in control
    assert "motion_abort" not in control
    assert "servo_move_enable" not in control

    assert 'service_prefix + "/robot_power"' in current
    assert 'service_prefix + "/robot_enable"' in current


def test_driver_build_metadata_contains_std_srvs_and_policy_test():
    cmake = read("src/jaka_ros2/src/jaka_driver/CMakeLists.txt")
    package = read("src/jaka_ros2/src/jaka_driver/package.xml")
    assert "find_package(std_srvs REQUIRED)" in cmake
    assert "std_srvs" in cmake
    assert "operator_state_policy_test" in cmake
    assert "<depend>std_srvs</depend>" in package
