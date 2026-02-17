#!/usr/bin/env python3
"""
export_trajectory.py
====================
Runs the PID-controlled SO101 simulation (same motion sequence as the lab)
and exports a time-stamped CSV with:

  time          – simulation time [s]
  q_des_<j>     – desired (commanded) joint position [rad]
  q_actual_<j>  – measured joint position from MuJoCo [rad]
  tau_<j>       – total applied torque (PID + disturbance) [N·m]

Output: trajectory_data/mujoco_trajectory.csv
        trajectory_data/mujoco_trajectory_meta.json  (run parameters)

Usage (from simulation_code/):
    python export_trajectory.py
"""

from __future__ import annotations
import os
import csv
import json
import time
import numpy as np
import mujoco

from so101_control import (
    JointPID, PIDGains,
    PerturbationModel, PerturbationConfig,
    get_q_qd_dict,
    apply_joint_torques_qfrc,
)
from so101_mujoco_pid_utils import (
    lerp_pose,
    build_default_pid,
    build_default_perturbations,
    DEFAULT_JOINTS,
)
from so101_mujoco_utils2 import set_initial_pose

# ─────────────────────────── CONFIG ────────────────────────────────────────

MODEL_PATH = "model/scene_urdf.xml"
OUT_DIR    = "trajectory_data"
OUT_CSV    = os.path.join(OUT_DIR, "mujoco_trajectory.csv")
OUT_META   = os.path.join(OUT_DIR, "mujoco_trajectory_meta.json")

JOINT_NAMES = DEFAULT_JOINTS   # ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]

# ── Motion identical to the lab demo ───────────────────────────────────────
STARTING_POSITION = {
    "shoulder_pan":  np.deg2rad(-4.4003158666),
    "shoulder_lift": np.deg2rad(-92.2462050161),
    "elbow_flex":    np.deg2rad(89.9543738355),
    "wrist_flex":    np.deg2rad(55.1185398916),
    "wrist_roll":    np.deg2rad(0.0),
}

ZERO_POSITION = {jn: 0.0 for jn in JOINT_NAMES}

# Sequence: (target_rad, duration_s, label)
MOTION_SEQUENCE = [
    (ZERO_POSITION,       2.0, "move_to_zero"),
    (ZERO_POSITION,       2.0, "hold_zero"),
    (STARTING_POSITION,   2.0, "return_to_start"),
    (STARTING_POSITION,   2.0, "hold_start"),
]

# ─────────────────────────── HELPERS ───────────────────────────────────────

def _run_segment(
    m, d,
    q_target: dict[str, float],
    duration: float,
    label: str,
    pid: JointPID,
    perturb: PerturbationModel,
    writer: csv.DictWriter,
    interpolate: bool = True,
):
    """Run one motion segment, logging every step."""
    pid.reset()

    q0, _ = get_q_qd_dict(m, d, JOINT_NAMES)
    q_des_final = q_target
    steps = int(max(1, duration / m.opt.timestep))
    t0 = float(d.time)

    for _ in range(steps):
        t  = float(d.time)
        s  = (t - t0) / max(duration, 1e-9)
        q_des = lerp_pose(q0, q_des_final, s) if interpolate else q_des_final

        q, qd = get_q_qd_dict(m, d, JOINT_NAMES)
        q_meas, qd_meas = perturb.noisy_measurement(q, qd)

        tau_pid  = pid.compute(q_meas, qd_meas, q_des, m.opt.timestep)
        tau_dist = perturb.apply_joint_torques(t=t, dt=m.opt.timestep)

        tau_total = {jn: tau_pid[jn] + tau_dist[jn] for jn in JOINT_NAMES}
        apply_joint_torques_qfrc(m, d, JOINT_NAMES, tau_total)
        mujoco.mj_step(m, d)

        # ── Build CSV row ──────────────────────────────────────────────────
        row: dict = {"time": t, "segment": label}
        for jn in JOINT_NAMES:
            row[f"q_des_{jn}"]    = q_des[jn]
            row[f"q_actual_{jn}"] = q[jn]
            row[f"tau_{jn}"]      = tau_total[jn]
        writer.writerow(row)


# ─────────────────────────── MAIN ──────────────────────────────────────────

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)

    # ── PID and perturbation setup ─────────────────────────────────────────
    pid    = build_default_pid(JOINT_NAMES)
    perturb = build_default_perturbations(JOINT_NAMES)

    # ── Set starting pose (no dynamics, instant) ───────────────────────────
    set_initial_pose(m, d, {
        jn: np.rad2deg(v) for jn, v in STARTING_POSITION.items()
    } | {"gripper": 0.0})

    # ── CSV header ─────────────────────────────────────────────────────────
    fieldnames = ["time", "segment"]
    for jn in JOINT_NAMES:
        fieldnames += [f"q_des_{jn}", f"q_actual_{jn}", f"tau_{jn}"]

    t_start_wall = time.time()

    with open(OUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for idx, (target, dur, label) in enumerate(MOTION_SEQUENCE):
            interpolate = not label.startswith("hold")
            print(f"  [{idx+1}/{len(MOTION_SEQUENCE)}] {label}  ({dur:.1f}s) …")
            _run_segment(m, d, target, dur, label, pid, perturb, writer, interpolate)

    wall_elapsed = time.time() - t_start_wall

    # ── Metadata ───────────────────────────────────────────────────────────
    meta = {
        "model_path": MODEL_PATH,
        "joint_names": JOINT_NAMES,
        "timestep_s": float(m.opt.timestep),
        "total_sim_time_s": float(d.time),
        "wall_time_s": wall_elapsed,
        "motion_sequence": [
            {"label": label, "duration_s": dur}
            for _, dur, label in MOTION_SEQUENCE
        ],
        "pid_gains": {
            jn: vars(pid.gains[jn]) for jn in JOINT_NAMES
        },
        "perturbation_config": {
            k: v for k, v in vars(perturb.cfg).items()
        },
    }
    with open(OUT_META, "w") as f:
        json.dump(meta, f, indent=2)

    print(f"\nDone. Exported {OUT_CSV}")
    print(f"  Sim time : {d.time:.3f} s  |  Wall time: {wall_elapsed:.1f} s")
    print(f"  Metadata : {OUT_META}")


if __name__ == "__main__":
    main()
