from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from dual_arm_app.backend.operator_runtime import (
    OperatorRuntime,
    RuntimeRejected,
    UserSystemd,
    actual_state,
    control_robot,
    driver_matches,
    normalize_unit,
    scan_processes,
    shutdown_decision,
    user_bus_env,
)


class FakeSystemd:
    def __init__(self, states=None):
        self.states = states or {}
        self.commands = []

    def status(self, key):
        state = self.states.get(key, "STOPPED")
        properties = {
            "LoadState": "loaded",
            "ActiveState": {
                "STOPPED": "inactive",
                "STARTING": "activating",
                "RUNNING": "active",
                "STOPPING": "deactivating",
                "ERROR": "failed",
            }[state],
            "ControlGroup": "/user.slice/jaka-operator-moveit.service" if key == "moveit" else "",
        }
        return {"state": state, "unit": f"fake-{key}.service", "properties": properties, "error": None}

    def command(self, action, key):
        self.commands.append((action, key))
        return ""


def cache(power=1, enabled=1, motion=0, received=9_500, valid=True):
    return {
        "valid": valid,
        "received_at_ms": received,
        "state": {"power_state": power, "servo_state": enabled, "motion_state": motion},
    }


def connected_driver(*, power=True, enabled=True, motion=0, fresh=True):
    return {
        "connection": "CONNECTED",
        "actual": {
            "fresh": fresh,
            "power": power if fresh else "UNKNOWN",
            "enabled": enabled if fresh else "UNKNOWN",
            "motion": motion if fresh else "UNKNOWN",
        },
    }


def terminal_feedback(state="IDLE"):
    return {"valid": True, "active": False, "state": state}


def test_user_bus_env_fills_missing_values_and_preserves_explicit_values():
    env = user_bus_env({}, uid=1234)
    assert env["XDG_RUNTIME_DIR"] == "/run/user/1234"
    assert env["DBUS_SESSION_BUS_ADDRESS"] == "unix:path=/run/user/1234/bus"
    explicit = user_bus_env({"XDG_RUNTIME_DIR": "/tmp/runtime", "DBUS_SESSION_BUS_ADDRESS": "unix:path=/tmp/bus"}, uid=2)
    assert explicit["XDG_RUNTIME_DIR"] == "/tmp/runtime"
    assert explicit["DBUS_SESSION_BUS_ADDRESS"] == "unix:path=/tmp/bus"


@pytest.mark.parametrize(
    ("active", "expected"),
    [("inactive", "STOPPED"), ("activating", "STARTING"), ("active", "RUNNING"),
     ("deactivating", "STOPPING"), ("failed", "ERROR"), ("mystery", "ERROR")],
)
def test_normalize_systemd_state(active, expected):
    assert normalize_unit({"LoadState": "loaded", "ActiveState": active}) == expected
    assert normalize_unit({"LoadState": "not-found", "ActiveState": active}) == "ERROR"


def test_user_systemd_rejects_arbitrary_units_actions_and_driver_stop_restart():
    calls = []

    def runner(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(returncode=0, stdout="LoadState=loaded\nActiveState=inactive\n", stderr="")

    systemd = UserSystemd(runner=runner)
    with pytest.raises(RuntimeRejected):
        systemd.command("start", "attacker.service")
    with pytest.raises(RuntimeRejected):
        systemd.command("exec", "web")
    with pytest.raises(RuntimeRejected):
        systemd.command("stop", "left")
    with pytest.raises(RuntimeRejected):
        systemd.command("restart", "right")
    systemd.command("start", "web")
    assert calls[-1][-1] == "jaka-operator-web.service"


def test_driver_matching_covers_fixed_forms_without_command_text_false_positives():
    left_native = ["/opt/jaka_driver", "--ros-args", "-p", "ip:=192.168.0.1"]
    right_ros2 = ["ros2", "run", "jaka_driver", "jaka_driver", "--ros-args", "-r", "__node:=right_jaka_driver_node"]
    assert driver_matches(left_native, "left")
    assert not driver_matches(left_native, "right")
    assert driver_matches(right_ros2, "right")
    assert driver_matches(["bash", "/home/u/jaka_ws/real_robot_scripts/02_left_driver.sh"], "left")
    assert driver_matches(["python3", "/home/u/jaka_ws/operator_runtime/driver_guard.py", "right"], "right")
    assert not driver_matches(["grep", "192.168.0.1", "left_jaka_driver_node"], "left")
    assert not driver_matches(["bash", "-lc", "ros2 run jaka_driver jaka_driver -p ip:=192.168.0.1"], "left")
    assert not driver_matches(["python3", "test.py", "driver_guard.py", "left"], "left")


def test_scan_processes_reads_argv_and_ignores_nonnumeric_and_empty(tmp_path):
    (tmp_path / "123").mkdir()
    (tmp_path / "123" / "cmdline").write_bytes(b"ros2\0run\0jaka_driver\0jaka_driver\0")
    (tmp_path / "124").mkdir()
    (tmp_path / "124" / "cmdline").write_bytes(b"")
    (tmp_path / "self").mkdir()
    result = scan_processes(tmp_path)
    assert result == [{"pid": 123, "argv": ["ros2", "run", "jaka_driver", "jaka_driver"]}]


def test_actual_state_requires_fresh_valid_robotmsg():
    fresh = actual_state(cache(), now_ms=10_000, stale_ms=1_500)
    assert fresh["fresh"] is True
    assert fresh["power"] is True and fresh["enabled"] is True and fresh["motion"] == 0
    stale = actual_state(cache(received=8_000), now_ms=10_000, stale_ms=1_500)
    assert stale["fresh"] is False
    assert stale["power"] == "UNKNOWN" and stale["motion"] == "UNKNOWN"
    invalid = actual_state(cache(valid=False), now_ms=10_000)
    assert invalid["fresh"] is False


def test_connect_starts_only_missing_side_and_never_duplicates_existing_driver():
    systemd = FakeSystemd()
    processes = [{"pid": 42, "argv": ["jaka_driver", "--ros-args", "-p", "ip:=192.168.0.1"]}]
    runtime = OperatorRuntime(systemd=systemd, process_reader=lambda: processes, clock=lambda: 1.0, wall_clock_ms=lambda: 10_000)
    result = runtime.connect(lambda: {"left": {}, "right": {}}, lambda: ())
    assert "no second session" in result["left"]
    assert "start requested" in result["right"]
    assert systemd.commands == [("start", "right")]


def test_expired_pending_start_can_be_retried():
    now = [0.0]
    systemd = FakeSystemd()
    runtime = OperatorRuntime(systemd=systemd, process_reader=lambda: [], clock=lambda: now[0], wall_clock_ms=lambda: 10_000)
    runtime.connect(lambda: {"left": {}, "right": {}}, lambda: ())
    assert systemd.commands == [("start", "left"), ("start", "right")]
    now[0] = 31.0
    runtime.connect(lambda: {"left": {}, "right": {}}, lambda: ())
    assert systemd.commands == [("start", "left"), ("start", "right"), ("start", "left"), ("start", "right")]


def test_fresh_actual_feedback_classifies_connected_even_for_external_driver():
    systemd = FakeSystemd()
    runtime = OperatorRuntime(systemd=systemd, process_reader=lambda: [], clock=lambda: 1.0, wall_clock_ms=lambda: 10_000)
    status = runtime.status({"left": cache(), "right": {}}, live_sides=())
    assert status["drivers"]["left"]["connection"] == "CONNECTED"
    assert status["drivers"]["left"]["duplicate_blocked"] is True
    assert status["drivers"]["right"]["connection"] == "OFFLINE"


def test_moveit_reset_refuses_external_process_and_restarts_managed_unit_only():
    systemd = FakeSystemd({"moveit": "RUNNING"})
    process = [{"pid": 77, "argv": ["move_group"]}]
    runtime = OperatorRuntime(systemd=systemd, process_reader=lambda: process,
                              cgroup_reader=lambda pid: "0::/user.slice/external.service\n")
    with pytest.raises(RuntimeRejected, match="External MoveIt"):
        runtime.reset_moveit()
    assert systemd.commands == []

    runtime = OperatorRuntime(systemd=systemd, process_reader=lambda: process,
                              cgroup_reader=lambda pid: "0::/user.slice/jaka-operator-moveit.service\n")
    assert runtime.reset_moveit()["success"] is True
    assert systemd.commands == [("restart", "moveit")]


class Request:
    data = None


class FakeClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return "future"


def test_control_robot_preconditions_and_successful_fixed_setbool_call():
    client = FakeClient()
    wait = lambda future, timeout: SimpleNamespace(success=True, message="ok")
    with pytest.raises(RuntimeRejected, match="Disable robot explicitly"):
        control_robot("left", "power", False, connected_driver(enabled=True), client, Request, wait)
    with pytest.raises(RuntimeRejected, match="Power on"):
        control_robot("left", "enable", True, connected_driver(power=False, enabled=False), client, Request, wait)
    with pytest.raises(RuntimeRejected, match="idle"):
        control_robot("left", "enable", False, connected_driver(motion=3), client, Request, wait)
    with pytest.raises(RuntimeRejected, match="fresh known actual state"):
        control_robot("left", "enable", False, connected_driver(fresh=False), client, Request, wait)
    result = control_robot("left", "enable", False, connected_driver(), client, Request, wait)
    assert result["success"] is True
    assert client.requests[-1].data is False


def test_control_robot_rejects_nonfixed_input_and_unavailable_service():
    with pytest.raises(RuntimeRejected):
        control_robot("left;rm", "enable", True, connected_driver(), FakeClient(), Request, lambda *_a, **_k: None)
    with pytest.raises(RuntimeRejected):
        control_robot("left", "enable", 1, connected_driver(), FakeClient(), Request, lambda *_a, **_k: None)
    with pytest.raises(RuntimeRejected, match="service unavailable"):
        control_robot("left", "enable", False, connected_driver(), FakeClient(False), Request, lambda *_a, **_k: None)


def test_shutdown_gate_is_safe_with_no_drivers():
    status = {"drivers": {"left": {"connection": "OFFLINE"}, "right": {"connection": "OFFLINE"}}, "process_scan_error": None}
    decision = shutdown_decision(status, {})
    assert decision == {"safe": True, "state": "IDLE", "reason": "No robot drivers present"}


def test_shutdown_gate_connected_idle_terminal_is_safe_and_fail_closed_otherwise():
    status = {
        "drivers": {"left": connected_driver(), "right": connected_driver()},
        "process_scan_error": None,
    }
    execution = {"left": terminal_feedback(), "right": terminal_feedback("COMPLETED")}
    assert shutdown_decision(status, execution)["safe"] is True

    moving = {**status, "drivers": {**status["drivers"], "right": connected_driver(motion=3)}}
    assert shutdown_decision(moving, execution)["state"] == "ACTIVE"
    active = {**execution, "left": {"valid": True, "active": True, "state": "RUNNING"}}
    assert shutdown_decision(status, active)["state"] == "ACTIVE"
    stale = {**status, "drivers": {**status["drivers"], "left": connected_driver(fresh=False)}}
    assert shutdown_decision(stale, execution)["state"] == "UNKNOWN"
    connecting = {**status, "drivers": {**status["drivers"], "left": {"connection": "CONNECTING"}}}
    assert shutdown_decision(connecting, execution)["state"] == "UNKNOWN"
    scan_error = {**status, "process_scan_error": "permission denied"}
    assert shutdown_decision(scan_error, execution)["state"] == "UNKNOWN"
    assert shutdown_decision(status, execution, local_active=True)["state"] == "ACTIVE"


def test_phase5_passive_execution_snapshot_never_reads_feedback_or_requests_abort():
    from dual_arm_app.backend.phase5_execution_coordinator import Phase5ExecutionCoordinator

    feedback_calls = []
    abort_calls = []
    runtime = Phase5ExecutionCoordinator(
        artifact_snapshot_getter=lambda: (None, 0, "test"),
        phase4_gate_getter=lambda _fingerprint: {},
        transport=SimpleNamespace(inspection_state=lambda: {"robot_connection": {"both_ready": True}}),
        safe_state_checker=lambda _side: (True, "ok"),
        start_match_checker=lambda _artifact: {"match": False},
        legacy_conflict_getter=lambda: [],
        stop_generation_getter=lambda: 0,
        abort_callback=lambda: abort_calls.append("abort"),
        feedback_getter=lambda trajectory_id: feedback_calls.append(trajectory_id) or {},
    )
    runtime._execution = {"state": "RUNNING", "trajectory_id": "p5-test", "nested": {"x": 1}}
    snapshot = runtime.passive_execution_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert feedback_calls == []
    assert abort_calls == []
    snapshot["state"] = "CHANGED"
    assert runtime.passive_execution_snapshot()["state"] == "RUNNING"
