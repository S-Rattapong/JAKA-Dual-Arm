#!/usr/bin/env python3
"""Terminal-owned HMI lifecycle. Driver units deliberately outlive the browser."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import urlopen
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dual_arm_app.backend.operator_runtime import UserSystemd, UNITS, user_bus_env, scan_processes, moveit_matches

URL = "http://127.0.0.1:8000"


def read_status():
    with urlopen(URL + "/api/system-control/status", timeout=3) as response:
        return json.load(response)


def safe_to_close(status):
    # The backend gate owns all RobotMsg/Phase5/process truth. In particular,
    # both drivers being absent is a known-safe case and needs no RobotMsg.
    return status.get("shutdown", {}).get("safe") is True


def shutdown_when_safe(systemd, reader=read_status, sleep=time.sleep):
    while True:
        try:
            if safe_to_close(reader()):
                systemd.command("stop", "web")
                systemd.command("stop", "moveit")
                return
        except Exception as error:
            print(f"Shutdown status unavailable: {error}", flush=True)
        print("Waiting: motion active or status unknown; keeping Web and MoveIt alive. No STOP sent.", flush=True)
        sleep(3)


def main():
    os.environ.update(user_bus_env())
    state = Path.home() / ".local/state/jaka-dual-arm"
    state.mkdir(parents=True, exist_ok=True)
    lock = (state / "operator-hmi.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("Operator HMI launcher already running", flush=True)
        return 1
    closing = False
    def close_requested(signum, frame):
        nonlocal closing
        closing = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, close_requested)
    systemd = UserSystemd()
    logs = browser = None
    managed = False
    try:
        moveit = systemd.status("moveit")
        if moveit["state"] != "RUNNING" and any(moveit_matches(p["argv"]) for p in scan_processes()):
            raise RuntimeError("External MoveIt detected; close it explicitly before operator launch")
        # Refuse adopting a manual backend on our fixed HTTP port.
        try:
            existing = read_status()
        except Exception:
            existing = None
        if existing and existing.get("software", {}).get("web", {}).get("source") != "operator":
            raise RuntimeError("Manual Web backend detected; operator launch refused")
        systemd.command("start", "moveit")
        managed = True
        deadline = time.monotonic() + 60
        while systemd.status("moveit")["state"] != "RUNNING":
            if closing or time.monotonic() > deadline:
                raise RuntimeError("MoveIt startup interrupted or timed out")
            time.sleep(0.5)
        systemd.command("start", "web")
        logs = subprocess.Popen(["journalctl", "--user", "--follow", "--lines=30",
                                 *[arg for unit in UNITS.values() for arg in ("--unit", unit)]],
                                env=user_bus_env(), start_new_session=True)
        deadline = time.monotonic() + 60
        while not closing:
            try:
                if read_status().get("software", {}).get("web", {}).get("source") == "operator":
                    break
            except Exception:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError("Web HTTP readiness timed out")
            time.sleep(0.5)
        if not closing:
            profile = state / "operator-firefox"
            profile.mkdir(exist_ok=True)
            browser = subprocess.Popen(["firefox", "--new-instance", "--no-remote", "--profile", str(profile), URL],
                                       env=user_bus_env(), start_new_session=True)
            while browser.poll() is None and not closing:
                time.sleep(0.5)
    except Exception as error:
        print(f"Operator launcher: {error}", flush=True)
    finally:
        if browser is not None and browser.poll() is None:
            browser.terminate()
        if managed:
            shutdown_when_safe(systemd)
        if logs is not None:
            logs.terminate()
            logs.wait(timeout=5)
        lock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
