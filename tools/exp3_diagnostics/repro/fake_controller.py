"""Offline fake JAKA port-10000 pusher on 127.0.0.1 (never touches a real controller).

Runs in its own process so its timing is independent of the receiver's GIL.
Sends length-prefixed JSON frames (~8.7 KB, like the real controller) at a fixed
period on two local ports and logs each send's CLOCK_MONOTONIC ns.
usage: fake_controller.py PORT_LEFT PORT_RIGHT PERIOD_MS DURATION_S LOGFILE
"""
import json, socket, sys, threading, time

port_l, port_r, period_ms, duration_s, logfile = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
TARGET = 8740


def frame(i):
    body = {"len": 0, "joint_actual_position": [10.0 + i * 0.001, 20, 30, 40, 50, 60],
            "joint_position": [10.0 + i * 0.001, 20, 30, 40, 50, 60],
            "position": [1, 2, 3, 4, 5, 6], "actual_position": [1, 2, 3, 4, 5, 6], "pad": ""}
    base = len(json.dumps(body, separators=(",", ":")).encode())
    body["pad"] = "x" * max(0, TARGET - base - 4)
    for _ in range(4):
        body["len"] = len(json.dumps(body, separators=(",", ":")).encode())
    data = json.dumps(body, separators=(",", ":")).encode()
    assert len(data) == body["len"]
    return data


log = {"left": [], "right": [], "accepts": {"left": 0, "right": 0}}
LOGF = open(logfile, "w", buffering=1)
LOGLOCK = threading.Lock()


def serve(side, port, start_evt):
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port)); srv.listen(4); srv.settimeout(0.01)
    conn = None
    start_evt.wait()
    t0 = time.monotonic_ns(); i = 0
    while time.monotonic_ns() - t0 < duration_s * 1e9:
        try:  # accept (re)connections without blocking the send schedule
            c, _ = srv.accept(); conn = c; LOGF.write(f"accept {side} {time.monotonic_ns()}\n")
        except (socket.timeout, BlockingIOError):
            pass
        due = t0 + int(i * period_ms * 1e6)
        now = time.monotonic_ns()
        if now < due:
            time.sleep((due - now) / 1e9)
        data = frame(i)
        ts = time.monotonic_ns()
        with LOGLOCK:
            LOGF.write(f"{side} {i} {ts}\n")
        if conn is not None:
            try:
                conn.sendall(data)
            except OSError:
                conn = None
        i += 1
    if conn: conn.close()
    srv.close()


evt = threading.Event()
ths = [threading.Thread(target=serve, args=(s, p, evt)) for s, p in (("left", port_l), ("right", port_r))]
for t in ths: t.start()
evt.set()
for t in ths: t.join()
LOGF.close()
