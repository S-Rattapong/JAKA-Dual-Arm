"""Operator process and state policy. No ROS imports and no startup side effects."""
from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path

SIDES = ("left", "right")
DRIVERS = {
    side: {"ip": f"192.168.0.{index}", "node": f"{side}_jaka_driver_node",
           "prefix": f"/{side}_jaka_driver", "unit": f"jaka-operator-{side}-driver.service",
           "script": f"0{index + 1}_{side}_operator_driver.sh",
           "dev_script": f"0{index + 1}_{side}_driver.sh"}
    for index, side in enumerate(SIDES, 1)
}
UNITS = {**{side: spec["unit"] for side, spec in DRIVERS.items()},
         "moveit": "jaka-operator-moveit.service", "web": "jaka-operator-web.service"}
TERMINAL = {"IDLE", "COMPLETED", "ABORTED", "REJECTED", "FAILED", "STOPPED"}


class RuntimeRejected(RuntimeError):
    pass


def user_bus_env(environ=None, uid=None):
    env = dict(os.environ if environ is None else environ)
    env["XDG_RUNTIME_DIR"] = env.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid() if uid is None else uid}"
    env["DBUS_SESSION_BUS_ADDRESS"] = env.get("DBUS_SESSION_BUS_ADDRESS") or f"unix:path={env['XDG_RUNTIME_DIR']}/bus"
    return env


def driver_matches(argv, side):
    spec = DRIVERS[side]
    if not argv:
        return False
    names = [Path(arg).name for arg in argv]
    # Match executable positions, never arbitrary text in grep, shells -c,
    # editors, test runners, or command strings.
    executable = names[0]
    scripts = {spec["script"], spec["dev_script"]}
    if executable in scripts:
        return True
    if executable in {"bash", "sh"} and len(names) >= 2 and names[1] in scripts:
        return True
    if (executable in {"python3", "python"} and len(names) >= 3
            and names[1] == "driver_guard.py" and argv[2] == side):
        return True

    native = executable == "jaka_driver"
    ros2_index = 0 if executable == "ros2" else (
        1 if executable in {"python3", "python"} and len(names) > 1 and names[1] == "ros2" else None
    )
    wrapper = (ros2_index is not None and len(argv) > ros2_index + 3
               and argv[ros2_index + 1:ros2_index + 4] == ["run", "jaka_driver", "jaka_driver"])
    if not (native or wrapper):
        return False
    # Any matching identity is enough to block a duplicate (also catches partial remaps).
    identities = {f"ip:={spec['ip']}", f"__node:={spec['node']}",
                  f"__name:={spec['node']}", f"service_prefix:={spec['prefix']}",
                  f"service_prefix:={spec['prefix'].lstrip('/')}"}
    return bool(identities.intersection(argv))


def moveit_matches(argv):
    if not argv:
        return False
    names = [Path(arg).name for arg in argv]
    if names[0] == "move_group":
        return True
    return ((names[0] == "ros2" or (names[0] in {"python3", "python"} and "ros2" in names[1:3]))
            and "launch" in argv and "jaka_a12_moveit_config" in argv)


def scan_processes(proc_root=Path("/proc"), exclude=()):
    processes = []
    excluded = {os.getpid(), *exclude}
    for entry in Path(proc_root).iterdir():
        if not entry.name.isdigit() or int(entry.name) in excluded:
            continue
        try:
            argv = [arg.decode(errors="replace") for arg in (entry / "cmdline").read_bytes().split(b"\0") if arg]
            # Zombies have no argv. Permission errors fail closed, disappearances are normal.
            if argv:
                processes.append({"pid": int(entry.name), "argv": argv})
        except (FileNotFoundError, ProcessLookupError):
            continue
    return processes


def normalize_unit(properties):
    active = properties.get("ActiveState", "unknown")
    if properties.get("LoadState") != "loaded":
        return "ERROR"
    return {"inactive": "STOPPED", "activating": "STARTING", "active": "RUNNING",
            "deactivating": "STOPPING", "failed": "ERROR"}.get(active, "ERROR")


class UserSystemd:
    def __init__(self, runner=subprocess.run):
        self.runner = runner

    def command(self, action, key):
        if action not in {"show", "start", "stop", "restart"} or key not in UNITS:
            raise RuntimeRejected("Only fixed operator units and actions are permitted")
        if action in {"stop", "restart"} and key in SIDES:
            raise RuntimeRejected("Operator runtime never stops or restarts drivers")
        args = ["systemctl", "--user", action]
        if action == "show":
            args += ["--property=LoadState,ActiveState,SubState,MainPID,ControlGroup"]
        else:
            args += ["--no-block"]
        result = self.runner([*args, UNITS[key]], env=user_bus_env(),
                             capture_output=True, text=True, timeout=8, check=False)
        if result.returncode:
            raise RuntimeRejected(result.stderr.strip() or f"systemctl {action} failed")
        return result.stdout

    def status(self, key):
        try:
            props = dict(line.split("=", 1) for line in self.command("show", key).splitlines() if "=" in line)
            state = normalize_unit(props)
            return {"state": state, "unit": UNITS[key], "properties": props,
                    "error": "Install operator services or inspect journalctl --user" if state == "ERROR" else None}
        except (OSError, subprocess.SubprocessError, RuntimeRejected) as error:
            return {"state": "ERROR", "unit": UNITS[key], "properties": {}, "error": str(error)}


def actual_state(cached, now_ms, stale_ms=1500):
    timestamp = cached.get("received_at_ms")
    age = now_ms - timestamp if isinstance(timestamp, (int, float)) else None
    fresh = (cached.get("valid") is True and age is not None and 0 <= age <= stale_ms)
    state = cached.get("state") or {}
    def bit(name):
        value = state.get(name)
        return bool(value) if fresh and type(value) is int and value in (0, 1) else "UNKNOWN"
    motion = state.get("motion_state")
    return {"fresh": fresh, "age_ms": age, "received_at_ms": timestamp,
            "stale_timeout_ms": stale_ms, "source": "RobotMsg",
            "power": bit("power_state"), "enabled": bit("servo_state"),
            "motion": motion if fresh and type(motion) is int and motion in range(5) else "UNKNOWN",
            "error": None if fresh else "Fresh RobotMsg required; inspect driver and ROS domain"}


class OperatorRuntime:
    def __init__(self, systemd=None, process_reader=scan_processes, clock=time.monotonic,
                 wall_clock_ms=lambda: time.time() * 1000,
                 cgroup_reader=lambda pid: Path(f"/proc/{pid}/cgroup").read_text()):
        self.systemd = systemd or UserSystemd()
        self.process_reader = process_reader
        self.clock = clock
        self.wall_clock_ms = wall_clock_ms
        self.cgroup_reader = cgroup_reader
        self.first_seen = {}
        self.pending = {}
        self.lock = threading.RLock()

    def status(self, cache, live_sides=(), now_ms=None, backend_running=True):
        with self.lock:
            return self._status(cache, live_sides, now_ms, backend_running)

    def _status(self, cache, live_sides, now_ms, backend_running):
        now = self.clock()
        scan_error = None
        try:
            processes = self.process_reader()
        except OSError as error:
            processes, scan_error = [], str(error)
        drivers = {}
        for side in SIDES:
            pids = [p["pid"] for p in processes if driver_matches(p["argv"], side)]
            actual = actual_state(cache.get(side, {}), self.wall_clock_ms() if now_ms is None else now_ms)
            unit = self.systemd.status(side)
            pending_since = self.pending.get(side)
            pending = pending_since is not None and now - pending_since < 30
            if pending_since is not None and not pending:
                self.pending.pop(side, None)
            evidence = bool(pids or side in live_sides
                            or unit["state"] in {"STARTING", "RUNNING", "STOPPING"}
                            or pending)
            if evidence:
                self.first_seen.setdefault(side, now)
            else:
                self.first_seen.pop(side, None)
            if actual["fresh"]:
                state = "CONNECTED"
                self.pending.pop(side, None)
            elif scan_error:
                state = "ERROR"
            elif evidence:
                state = "CONNECTING" if now - self.first_seen[side] < 30 else "ERROR"
            elif unit["properties"].get("ActiveState") == "failed":
                state = "ERROR"
            else:
                state = "OFFLINE"
            managed = unit["state"] in {"STARTING", "RUNNING", "STOPPING"}
            drivers[side] = {"connection": state, "actual": actual, "pids": pids,
                             "ros_driver_present": side in live_sides, "unit": unit,
                             "source": "operator" if managed else "external" if pids or side in live_sides or actual["fresh"] else "none",
                             "duplicate_blocked": evidence or actual["fresh"] or bool(scan_error),
                             "error": scan_error or (actual["error"] if state != "CONNECTED" else None)}
        software = {key: self.systemd.status(key) for key in ("moveit", "web")}
        for key, value in software.items():
            value["unit_state"] = value["state"]
            value["source"] = "operator" if value["state"] in {"STARTING", "RUNNING", "STOPPING"} else "none"
            if key == "web" and backend_running:
                value["state"] = "RUNNING"
                value["source"] = "operator" if os.environ.get("JAKA_OPERATOR_MODE") == "1" else "manual"
            if key == "moveit" and value["source"] == "none" and any(moveit_matches(p["argv"]) for p in processes):
                value.update(state="RUNNING", source="external")
        return {"drivers": drivers, "software": software, "process_scan_error": scan_error}

    def connect(self, cache_getter, live_getter):
        with self.lock:
            status = self.status(cache_getter(), live_getter())
            result = {}
            for side in SIDES:
                driver = status["drivers"][side]
                if driver["duplicate_blocked"]:
                    result[side] = "existing driver or pending start; no second session launched"
                    continue
                # A failed job can be retried only when no process/ROS evidence exists.
                self.systemd.command("start", side)
                self.pending[side] = self.clock()
                self.first_seen[side] = self.clock()
                result[side] = "start requested (connect + power + enable)"
            return result

    def reset_moveit(self):
        with self.lock:
            state = self.systemd.status("moveit")
            if state["state"] != "RUNNING":
                raise RuntimeRejected("Reset requires running operator-managed MoveIt; external Dev MoveIt is never restarted")
            group = state["properties"].get("ControlGroup")
            for process in self.process_reader():
                if moveit_matches(process["argv"]):
                    try:
                        membership = self.cgroup_reader(process["pid"])
                    except OSError as error:
                        raise RuntimeRejected("Cannot establish MoveIt ownership") from error
                    if not group or group == "/" or not any(line.split(":", 2)[-1] == group or
                                            line.split(":", 2)[-1].startswith(group + "/")
                                            for line in membership.splitlines()):
                        raise RuntimeRejected("External MoveIt detected; reset refused")
            # Only this fixed unit is affected, never pkill/killall or another launch.
            self.systemd.command("restart", "moveit")
            return {"success": True, "message": "Operator MoveIt restart requested"}


def control_robot(side, control, desired, driver, client, request_factory, wait_result):
    if side not in SIDES or control not in {"power", "enable"} or type(desired) is not bool:
        raise RuntimeRejected("Expected fixed side/control and a strict boolean")
    actual = driver["actual"]
    if driver["connection"] != "CONNECTED" or not actual["fresh"] or any(
            actual[name] == "UNKNOWN" for name in ("power", "enabled", "motion")):
        raise RuntimeRejected("Driver must be CONNECTED with fresh known actual state")
    if control == "power" and not desired and actual["enabled"] is not False:
        raise RuntimeRejected("Disable robot explicitly before power off")
    if control == "enable" and desired and actual["power"] is not True:
        raise RuntimeRejected("Power on before enabling robot")
    if not desired and actual["motion"] != 0:
        raise RuntimeRejected("Robot must be idle; this control never stops motion")
    if not client.service_is_ready():
        raise RuntimeRejected("State-control service unavailable; rebuild/restart driver when safe")
    request = request_factory()
    request.data = desired
    response = wait_result(client.call_async(request), timeout=5.0)
    if response is None:
        raise RuntimeRejected("SDK service outcome unknown after timeout; verify fresh actual state before retrying")
    return {"success": bool(response.success), "message": str(response.message),
            "actual_state_pending_feedback": bool(response.success)}


def shutdown_decision(status, execution, local_active=False):
    """No STOP side effects. Unknown always retains Web and MoveIt."""
    if local_active:
        return {"safe": False, "state": "ACTIVE", "reason": "Backend execution/command is active"}
    if status.get("process_scan_error"):
        return {"safe": False, "state": "UNKNOWN", "reason": "Process inspection failed"}
    connected = []
    for side in SIDES:
        driver = status["drivers"][side]
        connection = driver.get("connection")
        if connection == "OFFLINE":
            continue
        if connection != "CONNECTED":
            return {"safe": False, "state": "UNKNOWN", "reason": f"{side} driver state is {connection}"}
        connected.append(side)
        actual = driver["actual"]
        feedback = execution.get(side)
        if feedback and feedback.get("active") is True:
            return {"safe": False, "state": "ACTIVE", "reason": f"{side} Phase5 is active"}
        if not feedback or feedback.get("valid") is not True or feedback.get("active") is not False or feedback.get("state") not in TERMINAL:
            return {"safe": False, "state": "UNKNOWN", "reason": f"{side} Phase5 status unavailable/nonterminal"}
        if not actual["fresh"] or actual["motion"] == "UNKNOWN":
            return {"safe": False, "state": "UNKNOWN", "reason": f"{side} actual motion state is stale/unknown"}
        if actual["motion"] != 0:
            return {"safe": False, "state": "ACTIVE", "reason": f"{side} controller is not idle"}
    reason = "No robot drivers present" if not connected else "Connected drivers report terminal execution and fresh idle motion"
    return {"safe": True, "state": "IDLE", "reason": reason}
