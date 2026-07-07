#!/usr/bin/env python3
import json
import time
import urllib.request
import urllib.error

STATUS_URL = "http://localhost:8000/api/status"
POLL_SEC = 0.05


def get_status():
    with urllib.request.urlopen(STATUS_URL, timeout=0.5) as r:
        return json.loads(r.read().decode("utf-8"))


def motion_state(data, side):
    try:
        return int(data[side]["state"]["motion_state"])
    except Exception:
        return 0


def is_moving(data, side):
    return motion_state(data, side) != 0


def rel(t, start):
    if t is None:
        return None
    return round(t - start, 3)


def pick_motion_debug(active_motion):
    if not isinstance(active_motion, dict):
        return {}

    keys = [
        "type",
        "sync_enabled",
        "max_vel",
        "max_acc",
        "target_duration",
        "left_delta",
        "right_delta",
        "left_vel",
        "right_vel",
        "left_acc",
        "right_acc",
        "left_send",
        "right_send",
    ]

    return {k: active_motion.get(k) for k in keys if k in active_motion}


def pick_sequence_debug(active_sequence):
    if not isinstance(active_sequence, dict):
        return {}

    keys = [
        "status",
        "current_index",
        "current_name",
        "vel",
        "acc",
        "names",
    ]

    return {k: active_sequence.get(k) for k in keys if k in active_sequence}


def main():
    print("Dual JAKA Motion Finish Monitor V2")
    print("This version prints actual backend speed values.")
    print("Press Ctrl+C to stop.\n")

    run_id = 0
    in_motion = False

    start = None
    left_start = None
    right_start = None
    left_end = None
    right_end = None

    prev_left = False
    prev_right = False

    last_active_motion = {}
    last_active_sequence = {}

    while True:
        try:
            data = get_status()
        except Exception as e:
            print(f"[WARN] cannot read status: {e}")
            time.sleep(0.5)
            continue

        t = time.time()
        left = is_moving(data, "left")
        right = is_moving(data, "right")

        if isinstance(data.get("active_motion"), dict):
            last_active_motion = data.get("active_motion") or {}

        if isinstance(data.get("active_sequence"), dict):
            last_active_sequence = data.get("active_sequence") or {}

        if not in_motion and (left or right):
            run_id += 1
            in_motion = True

            start = t
            left_start = t if left else None
            right_start = t if right else None
            left_end = None
            right_end = None

            print(f"\n[Motion #{run_id}] started")
            print(f"  state: left={motion_state(data, 'left')} right={motion_state(data, 'right')}")

        if in_motion:
            if left and left_start is None:
                left_start = t
                print(f"  left started at {rel(t, start)} s")

            if right and right_start is None:
                right_start = t
                print(f"  right started at {rel(t, start)} s")

            if prev_left and not left and left_end is None:
                left_end = t
                print(f"  left stopped at {rel(t, start)} s")

            if prev_right and not right and right_end is None:
                right_end = t
                print(f"  right stopped at {rel(t, start)} s")

            if time.time() - start > 1.0:
                if left_start is None and left_end is None:
                    left_end = start
                if right_start is None and right_end is None:
                    right_end = start

            if left_end is not None and right_end is not None:
                finish_delta = round(abs(left_end - right_end), 3)

                print("\n" + "=" * 72)
                print(f"Motion #{run_id} finished")
                print("-" * 72)
                print(f"Left  started : {rel(left_start, start)} s")
                print(f"Right started : {rel(right_start, start)} s")
                print(f"Left  stopped : {rel(left_end, start)} s")
                print(f"Right stopped : {rel(right_end, start)} s")
                print(f"Finish delta  : {finish_delta} s")

                print("-" * 72)
                print("Sequence request values:")
                print(json.dumps(pick_sequence_debug(last_active_sequence), indent=2))

                print("-" * 72)
                print("Backend active_motion values:")
                print(json.dumps(pick_motion_debug(last_active_motion), indent=2))

                print("=" * 72 + "\n")

                in_motion = False

        prev_left = left
        prev_right = right
        time.sleep(POLL_SEC)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nMonitor stopped.")
