"""Read-only: per packet step, compare displacement of FK(joint_position) [URDF tip]
with displacement of the controller's own Cartesian actual_position (frame-invariant magnitudes).
If only the joint field were stale in a packet, the two step lengths would disagree there."""
import json, sys, numpy as np
sys.path.insert(0, "/home/newtonrattapong/jaka_ws")
from dual_arm_app.backend.experimental_rigid_grasp import UrdfFkModel, JOINT_NAMES, TERMINAL_LINKS
from dual_arm_app.backend.experimental_infrastructure import EXP2_URDF_PATH
rd = sys.argv[1]; watch = {"left": [37, 67, 118, 157], "right": [32, 116]}
raw = json.load(open(rd + "/raw.json")); model = UrdfFkModel(EXP2_URDF_PATH)
for side in ("left", "right"):
    pk = raw["actual_packets"][side]
    fk = np.array([np.array(model.fk(TERMINAL_LINKS[side], dict(zip(JOINT_NAMES[side], p["controller_joint_position_rad"]))))[:3, 3] * 1000 for p in pk])
    ca = np.array([p["controller_actual_position_raw"][:3] for p in pk])
    dfk = np.r_[0, np.linalg.norm(np.diff(fk, axis=0), axis=1)]; dca = np.r_[0, np.linalg.norm(np.diff(ca, axis=0), axis=1)]
    mv = dca > 3
    ratio = dfk[mv] / dca[mv]
    print(f"{side}: step-length ratio FK/controller over moving steps: median={np.median(ratio):.3f} p5..p95={np.percentile(ratio,5):.3f}..{np.percentile(ratio,95):.3f}")
    for i in watch[side]:
        seg = " ".join(f"[{j}] fk={dfk[j]:5.1f} ctl={dca[j]:5.1f}" for j in range(i - 1, min(i + 2, len(pk))))
        print(f"   around pkt{i}: {seg}")
