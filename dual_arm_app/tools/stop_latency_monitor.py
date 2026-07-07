#!/usr/bin/env python3
import json
import sys
import time
import urllib.request
import urllib.error

BASE_URL = "http://localhost:8000"
STATUS_URL = BASE_URL + "/api/status"

# หน่วงเวลาก่อนส่ง STOP หลัง detect ว่าหุ่นเริ่มขยับ
STOP_AFTER_SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
POLL_SEC = 0.03
TIMEOUT_SEC = 10.0


def get_status():
    with urllib.request.urlopen(STATUS_URL, timeout=0.5) as r:
        return json.loads(r.read().decode("utf-8"))


def motion_state(data, side):
    try:
        return int(data[side]["state"]["motion_state"])
    except Exception:
        return 0


def any_moving(data):
    return motion_state(data, "left") != 0 or motion_state(data, "right") != 0


def post_json(path, payload):
    url = BASE_URL + path
    body = json.dumps(payload).encode("utf-8")

    req = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with urllib.request.urlopen(req, timeout=1.0) as r:
        return json.loads(r.read().decode("utf-8"))


def send_stop():
    errors = []

    for path in ["/api/stop", "/api/sequence/stop"]:
        try:
            result = post_json(path, {"side": "both"})
            return path, result
        except Exception as e:
            errors.append(f"{path}: {e}")

    return None, {"ok": False, "errors": errors}


def main():
    print("Dual JAKA STOP Latency Monitor")
    print(f"Stop will be sent {STOP_AFTER_SEC:.2f} s after motion is detected.")
    print("Start this script first, then click Run Sequence on the web.")
    print("Press Ctrl+C to cancel.\n")

    # รอให้เริ่มจาก idle ก่อน
    while True:
        data = get_status()
        if not any_moving(data):
            break
        print("Waiting for robot to become idle...")
        time.sleep(0.5)

    print("Ready. Waiting for motion...\n")

    while True:
        data = get_status()

        if any_moving(data):
            start_t = time.time()
            print("Motion detected.")
            print(f"Initial state: left={motion_state(data, 'left')} right={motion_state(data, 'right')}")
            print(f"Waiting {STOP_AFTER_SEC:.2f} s before STOP...\n")
            break

        time.sleep(POLL_SEC)

    time.sleep(STOP_AFTER_SEC)

    stop_t = time.time()
    path, stop_result = send_stop()

    print("=" * 72)
    print("STOP command sent")
    print("-" * 72)
    print("Endpoint:", path)
    print("Response:")
    print(json.dumps(stop_result, indent=2, ensure_ascii=False))
    print("=" * 72 + "\n")

    while time.time() - stop_t < TIMEOUT_SEC:
        data = get_status()
        left_state = motion_state(data, "left")
        right_state = motion_state(data, "right")

        if left_state == 0 and right_state == 0:
            idle_t = time.time()
            latency = round(idle_t - stop_t, 3)

            print("=" * 72)
            print("STOP latency result")
            print("-" * 72)
            print(f"STOP latency : {latency} s")
            print(f"Final state  : left={left_state} right={right_state}")
            print("=" * 72)
            return

        time.sleep(POLL_SEC)

    print("=" * 72)
    print("STOP latency result")
    print("-" * 72)
    print("Timeout: robot did not become idle within timeout window")
    print("=" * 72)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled.")
