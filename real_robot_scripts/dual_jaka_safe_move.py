import subprocess
import sys
import time

MAX_ABS_DELTA = 0.02
VEL = 0.02
ACC = 0.05

LEFT_BASE = [
    2.8181028366088863,
    0.6005005240440368,
    -1.7843303680419924,
    1.3221564292907713,
    0.2156650573015213,
    1.0896085500717163,
]

RIGHT_BASE = [
    0.3300157189369201,
    2.253940582275391,
    1.5918420553207397,
    -1.3463665246963503,
    3.300674200057982,
    0.7286149859428406,
]

def check_delta(delta):
    if abs(delta) > MAX_ABS_DELTA:
        print(f"ERROR: delta {delta} exceeds MAX_ABS_DELTA={MAX_ABS_DELTA}")
        sys.exit(1)

def call_joint_move(prefix, joints):
    pose = "[" + ", ".join(f"{x:.10f}" for x in joints) + "]"
    service = prefix + "/joint_move"

    cmd = [
        "ros2", "service", "call",
        service,
        "jaka_msgs/srv/Move",
        "{pose: " + pose +
        ", has_ref: false, ref_joint: [], mvvelo: " + str(VEL) +
        ", mvacc: " + str(ACC) +
        ", mvtime: 0.0, mvradii: 0.0, coord_mode: 0, index: 0}"
    ]

    print(f"Calling {service}")
    print("Target:", joints)
    subprocess.run(cmd, check=False)

def stop_both():
    subprocess.run(["ros2", "service", "call", "/left_jaka_driver/stop_move", "std_srvs/srv/Empty", "{}"], check=False)
    subprocess.run(["ros2", "service", "call", "/right_jaka_driver/stop_move", "std_srvs/srv/Empty", "{}"], check=False)

def main():
    if len(sys.argv) != 3:
        print("Usage:")
        print("  python3 /tmp/dual_jaka_safe_move.py test 0.005")
        print("  python3 /tmp/dual_jaka_safe_move.py return 0")
        print("  python3 /tmp/dual_jaka_safe_move.py stop 0")
        sys.exit(1)

    mode = sys.argv[1]
    delta = float(sys.argv[2])

    if mode == "stop":
        stop_both()
        return

    if mode == "return":
        print("Returning both robots to baseline...")
        call_joint_move("/left_jaka_driver", LEFT_BASE)
        call_joint_move("/right_jaka_driver", RIGHT_BASE)
        return

    if mode == "test":
        check_delta(delta)

        left_target = LEFT_BASE[:]
        right_target = RIGHT_BASE[:]

        # Small opposite joint_6 motion
        left_target[5] += delta
        right_target[5] -= delta

        print("SAFE DUAL TEST")
        print(f"joint_6 delta = {delta:+.4f} rad")
        call_joint_move("/left_jaka_driver", left_target)
        call_joint_move("/right_jaka_driver", right_target)
        return

    print("ERROR: unknown mode:", mode)
    sys.exit(1)

if __name__ == "__main__":
    main()
