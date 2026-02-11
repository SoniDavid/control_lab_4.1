# so101_mujoco_pid_utils.py
from __future__ import annotations
import time
import numpy as np
import mujoco

from so101_control import (
    JointPID, PIDGains,
    PerturbationModel, PerturbationConfig,
    get_q_qd_dict,
    apply_joint_torques_qfrc,
)

# NEW: plotter type (optional; no hard dependency)
from typing import Protocol, Optional


class _PlotterProto(Protocol):
    def sample(self, m, d, now: float | None = None) -> None: ...


DEFAULT_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def lerp_pose(p0: dict[str, float], p1: dict[str, float], s: float) -> dict[str, float]:
    s = float(np.clip(s, 0.0, 1.0))
    out = {}
    for k in p0.keys():
        out[k] = (1.0 - s) * p0[k] + s * p1[k]
    return out


def build_default_pid(joint_names=DEFAULT_JOINTS) -> JointPID:
    # Conservative baseline gains; you will likely tune per joint.
    gains = {
        "shoulder_pan":  PIDGains(kp=0.0, ki=0.0, kd=0.0, i_limit=2.0, tau_limit=8.0),
        "shoulder_lift": PIDGains(kp=0.0, ki=0.0, kd=0.0, i_limit=2.0, tau_limit=18.0),
        "elbow_flex":    PIDGains(kp=0.0, ki=0.0, kd=0.5, i_limit=2.0, tau_limit=15.0),
        "wrist_flex":    PIDGains(kp=0.0, ki=0.0, kd=0.5, i_limit=2.0, tau_limit=6.0),
        "wrist_roll":    PIDGains(kp=0.0, ki=0.0, kd=0.8, i_limit=2.0, tau_limit=3.0),
    }
    # If some joints missing from dict, fallback
    for jn in joint_names:
        if jn not in gains:
            gains[jn] = PIDGains(kp=25.0, ki=0.3, kd=1.0, i_limit=2.0, tau_limit=6.0)
    return JointPID(joint_names, gains)


def build_default_perturbations(joint_names=DEFAULT_JOINTS) -> PerturbationModel:
    cfg = PerturbationConfig(
        sinus_amp=0.8,
        sinus_freq_hz=0.5,
        noise_std=0.25,
        noise_tau=0.25,
        impulse_prob_per_s=0.12,
        impulse_mag=2.0,
        impulse_dur=0.05,
        meas_q_std=0.0,
        meas_qd_std=0.0,
        seed=7
    )
    return PerturbationModel(joint_names, cfg)


def step_sim(m, d, viewer, realtime: bool, plotter: Optional[_PlotterProto] = None):
    """
    Single MuJoCo step + optional viewer sync + optional realtime pacing + optional plotting.
    We sample AFTER mj_step so the plotted state is the realized state.
    """
    mujoco.mj_step(m, d)

    if plotter is not None:
        plotter.sample(m, d)

    if viewer is not None:
        viewer.sync()

    if realtime:
        # keep it simple (you can swap for a more accurate wall-clock sync if desired)
        time.sleep(m.opt.timestep)


def move_to_pose_pid(
    m, d, viewer,
    target_pose_deg: dict[str, float],
    duration: float = 2.0,
    realtime: bool = True,
    joint_names=DEFAULT_JOINTS,
    pid: JointPID | None = None,
    perturb: PerturbationModel | None = None,
    # NEW:
    plotter: Optional[_PlotterProto] = None,
):
    """
    PID torque control with optional perturbations.
    Interpolates from current pose to target over 'duration' seconds.
    """
    if pid is None:
        pid = build_default_pid(joint_names)
    if perturb is None:
        perturb = build_default_perturbations(joint_names)

    pid.reset()

    # read initial q as "start" for interpolation
    q0, _ = get_q_qd_dict(m, d, joint_names)

    # convert target degrees -> radians (gripper ignored here)
    qT = {jn: np.deg2rad(target_pose_deg[jn]) for jn in joint_names}

    steps = int(max(1, duration / m.opt.timestep))
    t0 = float(d.time)

    for _ in range(steps):
        t = float(d.time)
        s = (t - t0) / max(duration, 1e-9)

        q_des = lerp_pose(q0, qT, s)

        q, qd = get_q_qd_dict(m, d, joint_names)
        q_meas, qd_meas = perturb.noisy_measurement(q, qd)

        tau_pid = pid.compute(q_meas, qd_meas, q_des, m.opt.timestep)
        tau_dist = perturb.apply_joint_torques(t=t, dt=m.opt.timestep)

        # Combine PID + perturbations
        tau_total = {jn: tau_pid[jn] + tau_dist[jn] for jn in joint_names}

        apply_joint_torques_qfrc(m, d, joint_names, tau_total)

        step_sim(m, d, viewer, realtime=realtime, plotter=plotter)


def hold_position_pid(
    m, d, viewer,
    hold_pose_deg: dict[str, float],
    duration: float = 2.0,
    realtime: bool = True,
    joint_names=DEFAULT_JOINTS,
    pid: JointPID | None = None,
    perturb: PerturbationModel | None = None,
    # NEW:
    plotter: Optional[_PlotterProto] = None,
):
    """
    Holds a fixed target pose with PID, injecting optional disturbances.
    """
    if pid is None:
        pid = build_default_pid(joint_names)
    if perturb is None:
        perturb = build_default_perturbations(joint_names)

    pid.reset()

    q_des = {jn: np.deg2rad(hold_pose_deg[jn]) for jn in joint_names}
    steps = int(max(1, duration / m.opt.timestep))

    for _ in range(steps):
        t = float(d.time)

        q, qd = get_q_qd_dict(m, d, joint_names)
        q_meas, qd_meas = perturb.noisy_measurement(q, qd)

        tau_pid = pid.compute(q_meas, qd_meas, q_des, m.opt.timestep)
        tau_dist = perturb.apply_joint_torques(t=t, dt=m.opt.timestep)

        tau_total = {jn: tau_pid[jn] + tau_dist[jn] for jn in joint_names}
        apply_joint_torques_qfrc(m, d, joint_names, tau_total)

        step_sim(m, d, viewer, realtime=realtime, plotter=plotter)
