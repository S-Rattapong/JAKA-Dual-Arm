#!/usr/bin/env python3
"""
Estimate dual-arm load sharing from static loaded/unloaded JAKA SDK torque logs.

This is an offline/read-only analyzer. It sends no robot commands.

Method:
  1) Median SDK-reported torque for unloaded and loaded static holds.
  2) Delta torque = loaded - unloaded.
  3) Parse the JAKA A12 URDF and compute a 6x6 geometric Jacobian.
  4) Estimate a TCP wrench with damped least squares:
         delta_tau = J.T @ wrench
  5) Use the magnitude of the estimated TCP force as a load-sharing indicator.

Important:
- Until SDK torque units are calibrated, the result is an Estimated Load Share,
  not a direct force measurement in newtons or kilograms.
- The force-share ratio is more useful than the absolute force magnitude.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


MOVABLE_TYPES = {"revolute", "continuous", "prismatic"}


def parse_vector(text: Optional[str], default: Sequence[float]) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=float)
    values = [float(x) for x in text.replace(",", " ").split()]
    if len(values) != len(default):
        raise ValueError(f"Expected {len(default)} values, got {values}")
    return np.asarray(values, dtype=float)


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float
    )


def rot_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        raise ValueError("Joint axis has zero length")
    a = axis / norm
    K = skew(a)
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def rot_rpy(rpy: np.ndarray) -> np.ndarray:
    """URDF fixed-axis RPY: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def transform(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray


@dataclass
class Chain:
    root: str
    tip: str
    joints: List[Joint]

    @property
    def movable(self) -> List[Joint]:
        return [j for j in self.joints if j.joint_type in MOVABLE_TYPES]


class URDFModel:
    def __init__(self, path: Path) -> None:
        self.path = path
        root = ET.parse(path).getroot()
        self.links = {elem.attrib["name"] for elem in root.findall("link")}
        self.joints: List[Joint] = []
        self.children: Dict[str, List[Joint]] = {}
        child_links = set()

        for elem in root.findall("joint"):
            name = elem.attrib["name"]
            joint_type = elem.attrib.get("type", "fixed")
            parent_elem = elem.find("parent")
            child_elem = elem.find("child")
            if parent_elem is None or child_elem is None:
                continue

            parent = parent_elem.attrib["link"]
            child = child_elem.attrib["link"]
            origin = elem.find("origin")
            xyz = parse_vector(
                origin.attrib.get("xyz") if origin is not None else None,
                [0.0, 0.0, 0.0],
            )
            rpy = parse_vector(
                origin.attrib.get("rpy") if origin is not None else None,
                [0.0, 0.0, 0.0],
            )
            axis_elem = elem.find("axis")
            axis = parse_vector(
                axis_elem.attrib.get("xyz") if axis_elem is not None else None,
                [1.0, 0.0, 0.0],
            )

            joint = Joint(name, joint_type, parent, child, xyz, rpy, axis)
            self.joints.append(joint)
            self.children.setdefault(parent, []).append(joint)
            child_links.add(child)

        roots = sorted(self.links - child_links)
        if not roots:
            raise ValueError("Could not determine a URDF root link")
        self.roots = roots

    def all_leaf_chains(self) -> List[Chain]:
        chains: List[Chain] = []

        def walk(root_link: str, current_link: str, path: List[Joint]) -> None:
            next_joints = self.children.get(current_link, [])
            if not next_joints:
                chains.append(Chain(root_link, current_link, list(path)))
                return
            for joint in next_joints:
                walk(root_link, joint.child, path + [joint])

        for root in self.roots:
            walk(root, root, [])
        return chains

    def choose_six_axis_chain(self) -> Chain:
        candidates = []
        for chain in self.all_leaf_chains():
            movable_count = len(chain.movable)
            if movable_count == 6:
                # Prefer the chain with the most fixed joints after J6 / greatest depth.
                candidates.append((len(chain.joints), chain))
        if not candidates:
            summary = [
                (c.root, c.tip, len(c.movable), len(c.joints))
                for c in self.all_leaf_chains()
            ]
            raise ValueError(
                "No URDF chain with exactly six movable joints was found. "
                f"Leaf chains: {summary}"
            )
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]


def geometric_jacobian(chain: Chain, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    movable = chain.movable
    if len(movable) != 6 or q.shape != (6,):
        raise ValueError("Expected six movable joints and six joint positions")

    T = np.eye(4, dtype=float)
    records: List[Tuple[str, np.ndarray, np.ndarray]] = []
    qi = 0

    for joint in chain.joints:
        # Parent link -> joint frame
        T = T @ transform(rot_rpy(joint.rpy), joint.xyz)

        if joint.joint_type in MOVABLE_TYPES:
            axis_base = T[:3, :3] @ (
                joint.axis / max(np.linalg.norm(joint.axis), 1e-12)
            )
            origin_base = T[:3, 3].copy()
            records.append((joint.joint_type, origin_base, axis_base))

            if joint.joint_type in {"revolute", "continuous"}:
                T = T @ transform(rot_axis_angle(joint.axis, float(q[qi])), np.zeros(3))
            elif joint.joint_type == "prismatic":
                T = T @ transform(np.eye(3), joint.axis * float(q[qi]))
            qi += 1

    p_tip = T[:3, 3].copy()
    J = np.zeros((6, 6), dtype=float)

    for i, (joint_type, p_joint, axis_base) in enumerate(records):
        if joint_type in {"revolute", "continuous"}:
            J[:3, i] = np.cross(axis_base, p_tip - p_joint)
            J[3:, i] = axis_base
        else:
            J[:3, i] = axis_base
            J[3:, i] = 0.0

    return J, T


def robust_side_values(df: pd.DataFrame, side: str) -> Tuple[np.ndarray, np.ndarray, int]:
    subset = df[df["side"].astype(str).str.lower() == side].copy()
    if subset.empty:
        raise ValueError(f"No rows found for side={side!r}")

    torque_cols = [f"torque_j{i}" for i in range(1, 7)]
    joint_cols = [f"joint_position_j{i}" for i in range(1, 7)]

    torque_rows = subset[torque_cols].dropna()
    joint_rows = subset[joint_cols].dropna()
    if torque_rows.empty or joint_rows.empty:
        raise ValueError(f"Missing torque or joint-position samples for {side}")

    tau = torque_rows.median(axis=0).to_numpy(dtype=float)
    q = joint_rows.median(axis=0).to_numpy(dtype=float)
    return tau, q, len(torque_rows)


def damped_wrench(J: np.ndarray, delta_tau: np.ndarray, damping: float) -> Tuple[np.ndarray, Dict[str, float]]:
    A = J.T
    singular_values = np.linalg.svd(A, compute_uv=False)
    smax = float(np.max(singular_values))
    smin = float(np.min(singular_values))
    condition = float("inf") if smin < 1e-12 else smax / smin

    lam = max(float(damping), 0.0)
    wrench = np.linalg.solve(A.T @ A + (lam ** 2) * np.eye(6), A.T @ delta_tau)
    reconstructed = A @ wrench
    residual = delta_tau - reconstructed

    metrics = {
        "condition_number": condition,
        "smallest_singular_value": smin,
        "largest_singular_value": smax,
        "damping": lam,
        "torque_residual_l2": float(np.linalg.norm(residual)),
        "torque_delta_l2": float(np.linalg.norm(delta_tau)),
        "relative_residual": float(
            np.linalg.norm(residual) / max(np.linalg.norm(delta_tau), 1e-12)
        ),
    }
    return wrench, metrics


def bootstrap_force_share(
    unloaded: pd.DataFrame,
    loaded: pd.DataFrame,
    side: str,
    J: np.ndarray,
    damping: float,
    rng: np.random.Generator,
    count: int,
) -> np.ndarray:
    torque_cols = [f"torque_j{i}" for i in range(1, 7)]
    u = unloaded[unloaded["side"].astype(str).str.lower() == side][torque_cols].dropna().to_numpy(float)
    l = loaded[loaded["side"].astype(str).str.lower() == side][torque_cols].dropna().to_numpy(float)
    results = np.empty(count, dtype=float)

    for i in range(count):
        u_sample = u[rng.integers(0, len(u), len(u))]
        l_sample = l[rng.integers(0, len(l), len(l))]
        delta = np.median(l_sample, axis=0) - np.median(u_sample, axis=0)
        wrench, _ = damped_wrench(J, delta, damping)
        results[i] = np.linalg.norm(wrench[:3])
    return results


def locate_urdf(explicit: Optional[str]) -> Path:
    if explicit:
        path = Path(explicit).expanduser()
        if not path.exists():
            raise FileNotFoundError(path)
        return path

    preferred = [
        Path.home() / "jaka_ws/src/jaka_ros2/src/jaka_description/urdf/jaka_a12.urdf",
        Path.home() / "jaka_ws/src/jaka_ros2/jaka_description/urdf/jaka_a12.urdf",
        Path.home() / "jaka_ws/install/jaka_description/share/jaka_description/urdf/jaka_a12.urdf",
    ]
    for path in preferred:
        if path.exists():
            return path

    matches = list((Path.home() / "jaka_ws").glob("**/jaka_a12.urdf"))
    if matches:
        return matches[0]
    raise FileNotFoundError(
        "Could not find jaka_a12.urdf automatically. Pass --urdf /full/path/jaka_a12.urdf"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--unloaded", required=True, help="Unloaded static_load_samples.csv")
    parser.add_argument("--loaded", required=True, help="Loaded static_load_samples.csv")
    parser.add_argument("--urdf", default=None, help="Path to jaka_a12.urdf")
    parser.add_argument("--damping", type=float, default=0.05)
    parser.add_argument("--bootstrap", type=int, default=2000)
    parser.add_argument("--output", default="load_sharing_analysis")
    args = parser.parse_args()

    unloaded_path = Path(args.unloaded).expanduser()
    loaded_path = Path(args.loaded).expanduser()
    output_dir = Path(args.output).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    unloaded = pd.read_csv(unloaded_path)
    loaded = pd.read_csv(loaded_path)
    urdf_path = locate_urdf(args.urdf)
    model = URDFModel(urdf_path)
    chain = model.choose_six_axis_chain()

    result: Dict[str, object] = {
        "method": "damped Jacobian transpose inversion using SDK-reported delta torque",
        "unloaded_file": str(unloaded_path),
        "loaded_file": str(loaded_path),
        "urdf": str(urdf_path),
        "selected_chain": {
            "root": chain.root,
            "tip": chain.tip,
            "all_joints": [j.name for j in chain.joints],
            "movable_joints": [j.name for j in chain.movable],
        },
        "damping": args.damping,
        "interpretation": (
            "Estimated load-sharing indicator. Absolute wrench units require SDK torque calibration."
        ),
        "sides": {},
    }

    force_magnitudes: Dict[str, float] = {}
    jacobians: Dict[str, np.ndarray] = {}

    for side in ("left", "right"):
        tau_u, q_u, n_u = robust_side_values(unloaded, side)
        tau_l, q_l, n_l = robust_side_values(loaded, side)
        q_error = q_l - q_u

        J, T_tip = geometric_jacobian(chain, q_u)
        delta_tau = tau_l - tau_u
        wrench, metrics = damped_wrench(J, delta_tau, args.damping)
        force_mag = float(np.linalg.norm(wrench[:3]))
        moment_mag = float(np.linalg.norm(wrench[3:]))

        jacobians[side] = J
        force_magnitudes[side] = force_mag
        result["sides"][side] = {
            "unloaded_samples": n_u,
            "loaded_samples": n_l,
            "joint_position_unloaded_rad": q_u.tolist(),
            "joint_position_loaded_rad": q_l.tolist(),
            "max_pose_difference_rad": float(np.max(np.abs(q_error))),
            "unloaded_median_torque_raw": tau_u.tolist(),
            "loaded_median_torque_raw": tau_l.tolist(),
            "delta_torque_raw": delta_tau.tolist(),
            "estimated_tcp_wrench_raw": {
                "force_xyz": wrench[:3].tolist(),
                "moment_xyz": wrench[3:].tolist(),
                "force_magnitude": force_mag,
                "moment_magnitude": moment_mag,
            },
            "jacobian": J.tolist(),
            "tip_pose_in_base": T_tip.tolist(),
            "solver_metrics": metrics,
        }

    total_force = force_magnitudes["left"] + force_magnitudes["right"]
    left_share = 100.0 * force_magnitudes["left"] / max(total_force, 1e-12)
    right_share = 100.0 - left_share

    # Bootstrap confidence interval for the force-share indicator.
    rng = np.random.default_rng(20260717)
    left_boot = bootstrap_force_share(
        unloaded, loaded, "left", jacobians["left"], args.damping, rng, args.bootstrap
    )
    right_boot = bootstrap_force_share(
        unloaded, loaded, "right", jacobians["right"], args.damping, rng, args.bootstrap
    )
    shares = 100.0 * left_boot / np.maximum(left_boot + right_boot, 1e-12)
    ci_low, ci_high = np.percentile(shares, [2.5, 97.5])

    result["load_share_percent"] = {
        "left": left_share,
        "right": right_share,
        "left_bootstrap_95_percent_ci": [float(ci_low), float(ci_high)],
        "right_bootstrap_95_percent_ci": [float(100.0 - ci_high), float(100.0 - ci_low)],
    }

    json_path = output_dir / "load_sharing_result.json"
    json_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    csv_path = output_dir / "load_sharing_summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "side",
                "force_magnitude_raw",
                "load_share_percent",
                "max_pose_difference_rad",
                "condition_number",
                "relative_torque_residual",
            ]
        )
        for side, share in (("left", left_share), ("right", right_share)):
            side_result = result["sides"][side]
            writer.writerow(
                [
                    side,
                    force_magnitudes[side],
                    share,
                    side_result["max_pose_difference_rad"],
                    side_result["solver_metrics"]["condition_number"],
                    side_result["solver_metrics"]["relative_residual"],
                ]
            )

    print("=== Estimated Dual-Arm Load Sharing ===")
    print(f"URDF chain : {chain.root} -> {chain.tip}")
    print("Joints     : " + ", ".join(j.name for j in chain.movable))
    print(f"Left share : {left_share:.2f}%")
    print(f"Right share: {right_share:.2f}%")
    print(f"Left 95% CI: {ci_low:.2f}% to {ci_high:.2f}%")
    print()
    print("WARNING: This is an SDK-torque/Jacobian load indicator.")
    print("Absolute N or kg values require calibration with known loads.")
    print(f"Saved JSON : {json_path}")
    print(f"Saved CSV  : {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
