#!/usr/bin/env python3
import json
import time
import urllib.request
import urllib.error

STATUS_URL = "http://localhost:8000/api/status"
POLL_SEC = 0.05


def now():
    return time.time()


def rel(t, start):
    if t is None:
        return None
    return round(t - start, 3)


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


def print_result(run_id, start, left_start, right_start, left_end, right_end):
    finish_delta = None
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
    print("=" * 72 + "\n")


def main():
    print("Dual JAKA Motion Finish Monitor")
    print("Reading:", STATUS_URL)
    print("Press Ctrl+C to stop.")
    print("Start this monitor first, then run waypoint/sequence from the web.\n")

    run_id = 0
    in_motion = False

    start = None
    left_start = None
    right_start = None
    left_end = None
    right_end = None

    prev_left = False
    prev_right = False

    while True:
        try:
            data = get_status()
        except urllib.error.URLError as e:
            print(f"[WARN] cannot read /api/status: {e}")
            time.sleep(0.5)
            continue
        except Exception as e:
            print(f"[WARN] status parse error: {e}")
            time.sleep(0.5)
            continue

        t = now()
        left = is_moving(data, "left")
        right = is_moving(data, "right")

        # detect new motion
        if not in_motion and (left or right):
            run_id += 1
            in_motion = True
            start = t
            left_start = t if left else None
            right_start = t if right else None
            left_end = None
            right_end = None

            print(f"[Motion #{run_id}] started")
            print(f"  initial state: left={motion_state(data, 'left')} right={motion_state(data, 'right')}")

        if in_motion:
            # detect side start if it started slightly later
            if left and left_start is None:
                left_start = t
                print(f"  left started at {rel(t, start)} s")

            if right and right_start is None:
                right_start = t
                print(f"  right started at {rel(t, start)} s")

            # detect side stop
            if prev_left and not left and left_end is None:
                left_end = t
                print(f"  left stopped at {rel(t, start)} s")

            if prev_right and not right and right_end is None:
                right_end = t
                print(f"  right stopped at {rel(t, start)} s")

            # if a side never moved in this motion, mark it as not used after 1 sec
            if time.time() - start > 1.0:
                if left_start is None and left_end is None:
                    left_end = start
                if right_start is None and right_end is None:
                    right_end = start

            # both finished
            if left_end is not None and right_end is not None:
                print_result(run_id, start, left_start, right_start, left_end, right_end)
                in_motion = False

        prev_left = left
        prev_right = right
        time.sleep(POLL_SEC)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nMonitor stopped.")
