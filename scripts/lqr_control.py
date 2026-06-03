# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""DDSM115 free-spin and analytical LQR model diagnostics.

The ``free-spin`` and ``lqr-model`` modes fix the articulation root above the
ground, leaving the wheel joints free. The ``lqr-model`` mode is only a
suspended-air sign and motor-model test; it cannot demonstrate balancing.
"""

from __future__ import annotations

"""Launch Isaac Sim first."""

import argparse
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Run a fixed-base DDSM115 current-driven free-spin test.")
parser.add_argument("--disable_fabric", action="store_true", default=False)
parser.add_argument("--num_envs", type=int, default=1, help="Accepted for compatibility; always one environment.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-Standup-v0")
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--real-time", action="store_true", default=False)
parser.add_argument("--control-decimation", type=int, default=1, help="Physics steps per action update.")
parser.add_argument("--base-height", type=float, default=0.50, help="Fixed base height above ground in meters.")
parser.add_argument(
    "--test-mode",
    choices=("free-spin", "lqr-model", "lqr-floor"),
    default="free-spin",
    help="Select constant-current free-spin, suspended LQR model diagnostic, or opt-in floor-contact LQR test.",
)
parser.add_argument("--motor-test-current", type=float, default=0.25, help="Desired current for both free-spinning wheels in A.")
parser.add_argument("--left-motor-test-current", type=float, default=None, help="Optional left-wheel current override in A.")
parser.add_argument("--right-motor-test-current", type=float, default=None, help="Optional right-wheel current override in A.")
parser.add_argument(
    "--artificial-pitch-deg",
    type=float,
    default=1.0,
    help="Artificial pitch used by suspended lqr-model mode. The fixed body itself remains upright.",
)
parser.add_argument(
    "--floor-initial-pitch-deg", "--initial-pitch-deg",
    dest="floor_initial_pitch_deg",
    type=float,
    default=1.0,
    help="Initial model-state pitch for lqr-floor mode; magnitude must be between 0.5 and 3 degrees.",
)
parser.add_argument("--floor-motors-disabled", action="store_true", help="Run the lqr-floor baseline with zero wheel current.")
parser.add_argument("--max-test-time-s", type=float, default=5.0, help="Stop lqr-floor diagnostics after this simulated duration.")
parser.add_argument("--floor-stop-pitch-deg", type=float, default=10.0, help="Stop lqr-floor diagnostics when |theta| exceeds this value.")
parser.add_argument(
    "--plot-signals", type=str, default="wheel_rpm,motor_current,motor_torque",
    help="Comma-separated plot groups/signals. Use 'all', 'none', or --list-signals.",
)
parser.add_argument("--plot-window", type=int, default=500, help="Samples retained in each live plot.")
parser.add_argument("--plot-every", type=int, default=5, help="Update plots every N control steps.")
parser.add_argument("--no-plot", action="store_true", default=False)
parser.add_argument("--list-signals", action="store_true", default=False)
parser.add_argument("--log-csv", type=str, default="logs/ddsm115_free_spin.csv", help="Empty string disables CSV.")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
args_cli.num_envs = 1
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Everything below runs after Isaac Sim has launched."""

import csv
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from isaaclab.envs import DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config
from scipy.linalg import solve_continuous_are
from TwoWheeledRobot.tasks.direct.twowheeledrobot.sim_params import (
    DDSM115_I_PEAK,
    GROUND_DYNAMIC_FRICTION,
    GROUND_STATIC_FRICTION,
)


M_B = 2.716
M_W = 0.543
I_B = 0.01296
I_W = 0.000423
L = 0.10698
R_WHEEL = 0.07049
G = 9.81
KT = 0.75

A_LQR = np.array(
    [
        [0.0, 1.0, 0.0, 0.0],
        [125.106, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [-9.153, 0.0, 0.0, 0.0],
    ]
)
B_LQR = np.array([[0.0], [-68.323], [0.0], [10.356]])
Q_LQR = np.diag([300.0, 30.0, 1.0, 5.0])
R_LQR = np.array([[1.0]])
P_LQR = solve_continuous_are(A_LQR, B_LQR, Q_LQR, R_LQR)
K_LQR = np.linalg.inv(R_LQR) @ B_LQR.T @ P_LQR


@dataclass(frozen=True)
class RobotState:
    """State exposed to the user controller, in physical units."""

    time_s: float
    projected_gravity_body: tuple[float, float, float]
    angular_velocity_body_rad_s: tuple[float, float, float]
    root_position_world_m: tuple[float, float, float]
    root_linear_velocity_world_m_s: tuple[float, float, float]
    cg_joint_position_rad: tuple[float, float, float, float]
    cg_joint_velocity_rad_s: tuple[float, float, float, float]
    wheel_position_rad: tuple[float, float]
    wheel_velocity_rad_s: tuple[float, float]
    raw_wheel_position_rad: tuple[float, float]
    raw_wheel_velocity_rad_s: tuple[float, float]


@dataclass(frozen=True)
class RobotAction:
    """Action returned by the user controller, in physical units."""

    cg_joint_position_rad: tuple[float, float, float, float]
    wheel_current_a: tuple[float, float]


# ---------------------------------------------------------------------------
# USER CONTROL SECTION
# ---------------------------------------------------------------------------
def compute_action(state: RobotState) -> RobotAction:
    """Return physical wheel-current commands for the selected diagnostic."""
    if args_cli.test_mode == "free-spin":
        left_current = args_cli.left_motor_test_current
        right_current = args_cli.right_motor_test_current
        wheel_current = (
            args_cli.motor_test_current if left_current is None else left_current,
            args_cli.motor_test_current if right_current is None else right_current,
        )
    else:
        if args_cli.test_mode == "lqr-model":
            theta = math.radians(args_cli.artificial_pitch_deg)
            theta_dot = 0.0
            wheel_position_m = 0.0
            wheel_velocity_m_s = 0.0
        else:
            theta = pitch_from_projected_gravity(state.projected_gravity_body)
            theta_dot = -state.angular_velocity_body_rad_s[0]
            wheel_position_m = 0.5 * sum(state.wheel_position_rad) * R_WHEEL
            wheel_velocity_m_s = 0.5 * sum(state.wheel_velocity_rad_s) * R_WHEEL
        lqr_state = np.array([theta, theta_dot, wheel_position_m, wheel_velocity_m_s])
        i_des = 0.0 if args_cli.floor_motors_disabled else float(-(K_LQR @ lqr_state)[0])
        wheel_current = (i_des, i_des)

    return RobotAction(
        cg_joint_position_rad=(0.0, 0.0, 0.0, 0.0),
        wheel_current_a=wheel_current,
    )


# ---------------------------------------------------------------------------
# END USER CONTROL SECTION
# ---------------------------------------------------------------------------


def _tuple(value: torch.Tensor) -> tuple[float, ...]:
    return tuple(float(v) for v in value.detach().cpu().tolist())


def pitch_from_projected_gravity(projected_gravity_body: tuple[float, float, float]) -> float:
    """Return model pitch about the wheel axle from body-frame gravity."""
    _, gravity_y, gravity_z = projected_gravity_body
    return math.atan2(gravity_y, -gravity_z)


def lqr_current_for_pitch_deg(pitch_deg: float) -> float:
    """Return the zero-rate, zero-position LQR current command."""
    state = np.array([math.radians(pitch_deg), 0.0, 0.0, 0.0])
    return float(-(K_LQR @ state)[0])


def print_lqr_diagnostic() -> None:
    """Print the analytical model and verify the artificial pitch sign test."""
    print("Extracted parameters:")
    print(f"  m_b={M_B}  m_w={M_W}  I_b={I_B}  I_w={I_W}")
    print(f"  l={L}  r={R_WHEEL}  g={G}  Kt={KT}")
    print("A =", A_LQR)
    print("B =", B_LQR)
    print("K =", K_LQR)
    positive_current = lqr_current_for_pitch_deg(1.0)
    negative_current = lqr_current_for_pitch_deg(-1.0)
    print(f"[SIGN TEST] theta=+1.0 deg -> i_des={positive_current:+.6f} A")
    print(f"[SIGN TEST] theta=-1.0 deg -> i_des={negative_current:+.6f} A")
    if positive_current == 0.0 or negative_current == 0.0 or positive_current * negative_current >= 0.0:
        raise RuntimeError("LQR pitch sign test failed: positive and negative pitch must produce opposite nonzero currents.")
    print("[SIGN TEST] PASS: positive and negative pitch produce opposite current commands.")


def read_state(env: Any, time_s: float) -> RobotState:
    """Read env_0 and convert the mirrored left wheel axis to forward-positive."""
    data = env.robot.data
    raw_pos = data.joint_pos[0, env._wheel_ids]
    raw_vel = data.joint_vel[0, env._wheel_ids]
    wheel_sign = torch.tensor((-1.0, 1.0), device=env.device)
    return RobotState(
        time_s=time_s,
        projected_gravity_body=_tuple(env.bno080.data.projected_gravity_b[0]),
        angular_velocity_body_rad_s=_tuple(env.bno080.data.ang_vel_b[0]),
        root_position_world_m=_tuple(data.root_pos_w[0]),
        root_linear_velocity_world_m_s=_tuple(data.root_lin_vel_w[0]),
        cg_joint_position_rad=_tuple(data.joint_pos[0, env._cg_ids]),
        cg_joint_velocity_rad_s=_tuple(data.joint_vel[0, env._cg_ids]),
        wheel_position_rad=_tuple(raw_pos * wheel_sign),
        wheel_velocity_rad_s=_tuple(raw_vel * wheel_sign),
        raw_wheel_position_rad=_tuple(raw_pos),
        raw_wheel_velocity_rad_s=_tuple(raw_vel),
    )


def action_to_env(env: Any, action: RobotAction) -> torch.Tensor:
    """Convert physical commands to the StandupEnv six-dimensional action."""
    cg_target = torch.tensor(action.cg_joint_position_rad, device=env.device).reshape(1, 4)
    cg_action = (2.0 * (cg_target - env._cg_joint_lo) / (env._cg_joint_hi - env._cg_joint_lo) - 1.0).clamp(-1.0, 1.0)
    requested_current = torch.tensor(action.wheel_current_a, device=env.device).reshape(1, 2)
    wheel_action = requested_current / env.cfg.wheel_current_max
    return torch.cat((cg_action, wheel_action), dim=1).clamp(-1.0, 1.0)


def read_motor_debug(env: Any) -> dict[str, tuple[float, float]]:
    """Read the shared DDSM115 model stages for environment 0."""
    return {
        "i_des": _tuple(env._wheel_i_des[0]),
        "i_cmd": _tuple(env._wheel_i_cmd[0]),
        "tau_current": _tuple(env._wheel_tau_current[0]),
        "tau_speed_limit": _tuple(env._wheel_tau_speed_limit[0]),
        "tau_actual": _tuple(env._wheel_torque_cmd[0]),
    }


def read_wheel_contact_forces(env: Any) -> tuple[float, float]:
    """Return left/right wheel net contact-force magnitudes in newtons."""
    forces = env.wheel_contacts.data.net_forces_w[0, env._wheel_contact_ids]
    return tuple(float(v) for v in torch.linalg.vector_norm(forces, dim=-1).detach().cpu().tolist())


def sample_to_row(state: RobotState, motor: dict[str, tuple[float, float]], contact_forces: tuple[float, float]) -> dict[str, float]:
    """Flatten state and action into named CSV/plot signals."""
    theta = pitch_from_projected_gravity(state.projected_gravity_body)
    theta_dot = -state.angular_velocity_body_rad_s[0]
    position = 0.5 * sum(state.wheel_position_rad) * R_WHEEL
    velocity = 0.5 * sum(state.wheel_velocity_rad_s) * R_WHEEL
    return {
        "time": state.time_s,
        "theta": theta,
        "theta_dot": theta_dot,
        "position": position,
        "velocity": velocity,
        "i_des_left": motor["i_des"][0], "i_des_right": motor["i_des"][1],
        "i_cmd_left": motor["i_cmd"][0], "i_cmd_right": motor["i_cmd"][1],
        "tau_actual_left": motor["tau_actual"][0], "tau_actual_right": motor["tau_actual"][1],
        "wheel_rpm_left": state.wheel_velocity_rad_s[0] * 60.0 / (2.0 * math.pi),
        "wheel_rpm_right": state.wheel_velocity_rad_s[1] * 60.0 / (2.0 * math.pi),
        "base_position_x": state.root_position_world_m[0],
        "base_position_z": state.root_position_world_m[2],
        "contact_force_left": contact_forces[0], "contact_force_right": contact_forces[1],
        "current_saturated_left": float(abs(motor["i_des"][0] - motor["i_cmd"][0]) > 1.0e-6),
        "current_saturated_right": float(abs(motor["i_des"][1] - motor["i_cmd"][1]) > 1.0e-6),
        "time_s": state.time_s,
        "left_wheel_position_rad": state.wheel_position_rad[0],
        "right_wheel_position_rad": state.wheel_position_rad[1],
        "left_wheel_velocity_rad_s": state.wheel_velocity_rad_s[0],
        "right_wheel_velocity_rad_s": state.wheel_velocity_rad_s[1],
        "left_wheel_rpm": state.wheel_velocity_rad_s[0] * 60.0 / (2.0 * math.pi),
        "right_wheel_rpm": state.wheel_velocity_rad_s[1] * 60.0 / (2.0 * math.pi),
        "raw_left_wheel_velocity_rad_s": state.raw_wheel_velocity_rad_s[0],
        "raw_right_wheel_velocity_rad_s": state.raw_wheel_velocity_rad_s[1],
        "left_i_des_a": motor["i_des"][0], "right_i_des_a": motor["i_des"][1],
        "left_i_cmd_a": motor["i_cmd"][0], "right_i_cmd_a": motor["i_cmd"][1],
        "left_tau_current_nm": motor["tau_current"][0], "right_tau_current_nm": motor["tau_current"][1],
        "left_tau_speed_limit_nm": motor["tau_speed_limit"][0], "right_tau_speed_limit_nm": motor["tau_speed_limit"][1],
        "left_tau_actual_nm": motor["tau_actual"][0], "right_tau_actual_nm": motor["tau_actual"][1],
        "root_x_m": state.root_position_world_m[0], "root_y_m": state.root_position_world_m[1], "root_z_m": state.root_position_world_m[2],
        "root_vx_m_s": state.root_linear_velocity_world_m_s[0], "root_vy_m_s": state.root_linear_velocity_world_m_s[1], "root_vz_m_s": state.root_linear_velocity_world_m_s[2],
        "gravity_body_x": state.projected_gravity_body[0], "gravity_body_y": state.projected_gravity_body[1], "gravity_body_z": state.projected_gravity_body[2],
        "angular_velocity_body_x_rad_s": state.angular_velocity_body_rad_s[0],
        "angular_velocity_body_y_rad_s": state.angular_velocity_body_rad_s[1],
        "angular_velocity_body_z_rad_s": state.angular_velocity_body_rad_s[2],
    }


PLOT_SIGNALS: dict[str, tuple[str, str]] = {
    "left_wheel_velocity_rad_s": ("Left wheel velocity", "rad/s"), "right_wheel_velocity_rad_s": ("Right wheel velocity", "rad/s"),
    "left_wheel_rpm": ("Left wheel speed", "rpm"), "right_wheel_rpm": ("Right wheel speed", "rpm"),
    "left_wheel_position_rad": ("Left wheel position", "rad"), "right_wheel_position_rad": ("Right wheel position", "rad"),
    "left_i_des_a": ("Left desired current", "A"), "right_i_des_a": ("Right desired current", "A"),
    "left_i_cmd_a": ("Left saturated current", "A"), "right_i_cmd_a": ("Right saturated current", "A"),
    "left_tau_current_nm": ("Left current torque", "Nm"), "right_tau_current_nm": ("Right current torque", "Nm"),
    "left_tau_speed_limit_nm": ("Left torque-speed limit", "Nm"), "right_tau_speed_limit_nm": ("Right torque-speed limit", "Nm"),
    "left_tau_actual_nm": ("Left actual motor torque", "Nm"), "right_tau_actual_nm": ("Right actual motor torque", "Nm"),
    "root_x_m": ("Base X position", "m"), "root_y_m": ("Base Y position", "m"), "root_z_m": ("Base Z position", "m"),
    "root_vx_m_s": ("Base X velocity", "m/s"), "root_vy_m_s": ("Base Y velocity", "m/s"), "root_vz_m_s": ("Base Z velocity", "m/s"),
    "gravity_body_x": ("Body gravity X", ""), "gravity_body_y": ("Body gravity Y", ""), "gravity_body_z": ("Body gravity Z", ""),
    "angular_velocity_body_x_rad_s": ("Body angular velocity X", "rad/s"),
    "angular_velocity_body_y_rad_s": ("Body angular velocity Y", "rad/s"),
    "angular_velocity_body_z_rad_s": ("Body angular velocity Z", "rad/s"),
}
PLOT_GROUPS = {
    "wheel_velocity": ("left_wheel_velocity_rad_s", "right_wheel_velocity_rad_s"),
    "wheel_rpm": ("left_wheel_rpm", "right_wheel_rpm"),
    "wheel_position": ("left_wheel_position_rad", "right_wheel_position_rad"),
    "motor_current": ("left_i_des_a", "left_i_cmd_a", "right_i_des_a", "right_i_cmd_a"),
    "motor_torque": ("left_tau_current_nm", "left_tau_speed_limit_nm", "left_tau_actual_nm", "right_tau_current_nm", "right_tau_speed_limit_nm", "right_tau_actual_nm"),
    "base_position": ("root_x_m", "root_y_m", "root_z_m"), "base_velocity": ("root_vx_m_s", "root_vy_m_s", "root_vz_m_s"),
    "gravity": ("gravity_body_x", "gravity_body_y", "gravity_body_z"),
    "angular_velocity": ("angular_velocity_body_x_rad_s", "angular_velocity_body_y_rad_s", "angular_velocity_body_z_rad_s"),
}


def resolve_plot_signals(value: str) -> list[str]:
    requested = [item.strip() for item in value.split(",") if item.strip()]
    if not requested or requested == ["none"]:
        return []
    if "all" in requested:
        return list(PLOT_SIGNALS)
    resolved: list[str] = []
    for item in requested:
        for name in PLOT_GROUPS.get(item, (item,)):
            if name not in PLOT_SIGNALS:
                raise ValueError(f"Unknown plot signal/group '{name}'. Run with --list-signals.")
            if name not in resolved:
                resolved.append(name)
    return resolved


class CsvLogger:
    def __init__(self, path_value: str):
        self._file = self._writer = None
        if path_value:
            path = Path(path_value)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = path.open("w", newline="")
            print(f"[INFO]: Writing experiment CSV to {path}")

    def write(self, row: dict[str, float]) -> None:
        if self._file is None:
            return
        if self._writer is None:
            self._writer = csv.DictWriter(self._file, fieldnames=list(row))
            self._writer.writeheader()
        self._writer.writerow(row)

    def close(self) -> None:
        if self._file is not None:
            self._file.close()


class LivePlot:
    """Matplotlib window with one clearly titled subplot per selected signal."""
    def __init__(self, signals: list[str], window_size: int):
        import matplotlib.pyplot as plt
        self._plt, self._signals = plt, signals
        self._history = {name: deque(maxlen=window_size) for name in ("time_s", *signals)}
        plt.ion()
        self._figure, axes = plt.subplots(len(signals), 1, sharex=True, figsize=(9, max(3, 2.2 * len(signals))))
        self._axes = [axes] if len(signals) == 1 else list(axes)
        self._figure.canvas.manager.set_window_title("Two-wheeled robot state/action signals")
        self._lines = {}
        for axis, name in zip(self._axes, signals, strict=True):
            title, unit = PLOT_SIGNALS[name]
            (self._lines[name],) = axis.plot([], [], label=title)
            axis.set_title(title); axis.set_ylabel(unit); axis.grid(True, alpha=0.3); axis.legend(loc="upper left")
        self._axes[-1].set_xlabel("Time [s]")
        self._figure.tight_layout(); plt.show(block=False)

    def update(self, row: dict[str, float]) -> None:
        for name in self._history:
            self._history[name].append(row[name])
        times = list(self._history["time_s"])
        for axis, name in zip(self._axes, self._signals, strict=True):
            self._lines[name].set_data(times, list(self._history[name])); axis.relim(); axis.autoscale_view()
        self._figure.canvas.draw_idle(); self._figure.canvas.flush_events()

    def close(self) -> None:
        self._plt.close(self._figure)


def print_signal_names() -> None:
    print("Plot groups:")
    for name, members in PLOT_GROUPS.items():
        print(f"  {name}: {', '.join(members)}")
    print("Individual plot signals:")
    for name in PLOT_SIGNALS:
        print(f"  {name}")


def force_upright_fixed_spawn(env: Any, base_height: float) -> None:
    """Make StandupEnv resets place the fixed root upright at the test height."""
    def fixed_pose(count: int) -> tuple[torch.Tensor, torch.Tensor]:
        quat = torch.zeros(count, 4, device=env.device)
        quat[:, 0] = 1.0
        height = torch.full((count,), base_height, device=env.device)
        return quat, height

    env._sample_fallen_poses = fixed_pose


def force_floor_pitch_spawn(env: Any, pitch_deg: float) -> None:
    """Make StandupEnv resets start on the floor at a small pitch angle."""
    pitch_rad = math.radians(pitch_deg)

    def floor_pose(count: int) -> tuple[torch.Tensor, torch.Tensor]:
        quat = torch.zeros(count, 4, device=env.device)
        quat[:, 0] = math.cos(0.5 * pitch_rad)
        quat[:, 1] = -math.sin(0.5 * pitch_rad)
        height = torch.full((count,), env.cfg.spawn_upright_z, device=env.device)
        return quat, height

    env._sample_fallen_poses = floor_pose


def create_base_hold_joint(base_height: float) -> None:
    """Create a world-to-platform fixed joint for this CAD hierarchy."""
    import omni.usd
    from pxr import Gf, Sdf, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    platform_path = Sdf.Path(
        "/World/envs/env_0/Robot/SimplifiedBipedMainAssembly/"
        "SimplifiedBipedMainAssembly/Platform_Group"
    )
    platform_prim = stage.GetPrimAtPath(platform_path)
    if not platform_prim or not platform_prim.IsValid() or not platform_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"Base rigid body not found at {platform_path}")

    joint = UsdPhysics.FixedJoint.Define(stage, "/World/envs/env_0/BaseHoldJoint")
    joint.CreateBody1Rel().SetTargets([platform_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, base_height))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


@hydra_task_config(args_cli.task, None)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg: Any | None):
    if args_cli.list_signals:
        print_signal_names(); return
    if args_cli.test_mode == "lqr-floor" and not 0.5 <= abs(args_cli.floor_initial_pitch_deg) <= 3.0:
        raise ValueError("--floor-initial-pitch-deg magnitude must be between 0.5 and 3 degrees.")
    if args_cli.test_mode in ("lqr-model", "lqr-floor"):
        print_lqr_diagnostic()
    signals = [] if args_cli.no_plot else resolve_plot_signals(args_cli.plot_signals)
    env_cfg.scene.num_envs = 1
    env_cfg.decimation = args_cli.control_decimation
    env_cfg.sim.render_interval = args_cli.control_decimation
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else env_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device
    env_cfg.success_steps_required = 1_000_000_000; env_cfg.episode_length_s = 3600.0
    env_cfg.wheel_damping_scale_range = (1.0, 1.0)
    env_cfg.noise_proj_grav_std = 0.0; env_cfg.noise_ang_vel_std = 0.0; env_cfg.noise_cg_pos_std = 0.0
    left_current = args_cli.motor_test_current if args_cli.left_motor_test_current is None else args_cli.left_motor_test_current
    right_current = args_cli.motor_test_current if args_cli.right_motor_test_current is None else args_cli.right_motor_test_current
    requested_current_limit = max(abs(left_current), abs(right_current))
    if args_cli.test_mode == "lqr-model":
        requested_current_limit = abs(lqr_current_for_pitch_deg(args_cli.artificial_pitch_deg))
    elif args_cli.test_mode == "lqr-floor":
        requested_current_limit = 100.0
    # Permit the test to request more than I_PEAK so the shared model's current
    # saturation can be observed instead of being hidden by action normalization.
    env_cfg.wheel_current_max = max(DDSM115_I_PEAK, requested_current_limit)
    env_cfg.robot_cfg.init_state.pos = (0.0, 0.0, args_cli.base_height)
    if args_cli.test_mode == "lqr-floor":
        env_cfg.enable_wheel_contacts = True
        env_cfg.robot_cfg.spawn.activate_contact_sensors = True

    env = gym.make(args_cli.task, cfg=env_cfg)
    contact_names: list[str] = []
    if args_cli.test_mode == "lqr-floor":
        contact_ids, contact_names = env.unwrapped.wheel_contacts.find_bodies(
            ["DDSM115_Simplified", "DDSM115_Simplified_01"], preserve_order=True,
        )
        if len(contact_ids) != 2:
            raise RuntimeError(f"Expected two wheel contact bodies, found {contact_names}")
        env.unwrapped._wheel_contact_ids = torch.tensor(contact_ids, device=env.unwrapped.device, dtype=torch.long)
        force_floor_pitch_spawn(env.unwrapped, args_cli.floor_initial_pitch_deg)
    else:
        create_base_hold_joint(args_cli.base_height)
        force_upright_fixed_spawn(env.unwrapped, args_cli.base_height)
    env.reset()
    dt = env.unwrapped.step_dt
    logger, plotter = CsvLogger(args_cli.log_csv), None
    if signals:
        try:
            plotter = LivePlot(signals, args_cli.plot_window)
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN]: Live plot disabled: {exc}")
    print(f"[INFO]: DDSM115 test mode: {args_cli.test_mode}. Press Ctrl+C to stop.")
    print(f"[INFO]: dt={dt:.5f} s  rate={1.0 / dt:.1f} Hz")
    if args_cli.test_mode == "free-spin":
        print(f"[INFO]: fixed base height={args_cli.base_height:.3f} m")
        print(f"[INFO]: desired current left={left_current:+.3f} A  right={right_current:+.3f} A")
    elif args_cli.test_mode == "lqr-model":
        print(f"[INFO]: fixed base height={args_cli.base_height:.3f} m")
        print(f"[INFO]: artificial pitch={args_cli.artificial_pitch_deg:+.3f} deg")
        print("[INFO]: Suspended-air diagnostic only: do not claim floor-balancing performance from this run.")
    else:
        baseline = " motors-disabled baseline" if args_cli.floor_motors_disabled else " LQR enabled"
        print(f"[WARN]: FLOOR-CONTACT{baseline} at initial model pitch={args_cli.floor_initial_pitch_deg:+.3f} deg")
        import omni.usd
        base_hold_joint = omni.usd.get_context().get_stage().GetPrimAtPath("/World/envs/env_0/BaseHoldJoint")
        print(f"[CHECK]: base hold joint created: {bool(base_hold_joint and base_hold_joint.IsValid())}")
        print(f"[CHECK]: gravity enabled: {not env.unwrapped.cfg.robot_cfg.spawn.rigid_props.disable_gravity}")
        print(f"[CHECK]: ground friction enabled: static={GROUND_STATIC_FRICTION:.3f}, dynamic={GROUND_DYNAMIC_FRICTION:.3f}")
        print(f"[CHECK]: wheel contact bodies: {contact_names}")

    timestep, wall_start = 0, time.time()
    try:
        while simulation_app.is_running():
            start = time.time()
            with torch.inference_mode():
                state = read_state(env.unwrapped, timestep * dt)
                action = compute_action(state)
                env_action = action_to_env(env.unwrapped, action)
                env.step(env_action)
                state = read_state(env.unwrapped, (timestep + 1) * dt)
                motor = read_motor_debug(env.unwrapped)
                contact_forces = read_wheel_contact_forces(env.unwrapped) if args_cli.test_mode == "lqr-floor" else (0.0, 0.0)
            timestep += 1
            row = sample_to_row(state, motor, contact_forces); logger.write(row)
            if timestep == 1 and args_cli.test_mode in ("lqr-model", "lqr-floor"):
                print(
                    "[DDSM115 MODEL] "
                    f"i_des=[{motor['i_des'][0]:+.6f}, {motor['i_des'][1]:+.6f}] A  "
                    f"i_cmd=[{motor['i_cmd'][0]:+.6f}, {motor['i_cmd'][1]:+.6f}] A  "
                    f"tau_current=[{motor['tau_current'][0]:+.6f}, {motor['tau_current'][1]:+.6f}] Nm  "
                    f"tau_actual=[{motor['tau_actual'][0]:+.6f}, {motor['tau_actual'][1]:+.6f}] Nm"
                )
                if args_cli.test_mode == "lqr-floor":
                    print(
                        f"[CHECK]: wheel contact forces left={contact_forces[0]:.3f} N  right={contact_forces[1]:.3f} N  "
                        f"touching_floor={contact_forces[0] > 0.1 and contact_forces[1] > 0.1}"
                    )
                    print(
                        f"[CHECK]: equal current command={abs(motor['i_des'][0] - motor['i_des'][1]) < 1.0e-6}  "
                        f"current saturation logged=True"
                    )
            if plotter is not None and timestep % args_cli.plot_every == 0:
                plotter.update(row)
            if args_cli.test_mode == "lqr-floor":
                theta_deg = math.degrees(row["theta"])
                if abs(theta_deg) >= args_cli.floor_stop_pitch_deg:
                    print(f"[STOP]: |theta| reached {abs(theta_deg):.3f} deg; stopping floor diagnostic.")
                    break
                if row["time"] >= args_cli.max_test_time_s:
                    print(f"[STOP]: reached max floor-test time {args_cli.max_test_time_s:.3f} s.")
                    break
            if timestep % 100 == 0:
                print(
                    f"[step {timestep:6d}] rpm=[{row['left_wheel_rpm']:+.1f}, {row['right_wheel_rpm']:+.1f}]  "
                    f"omega=[{state.wheel_velocity_rad_s[0]:+.3f}, {state.wheel_velocity_rad_s[1]:+.3f}] rad/s  "
                    f"i_des=[{motor['i_des'][0]:+.2f}, {motor['i_des'][1]:+.2f}] A  "
                    f"i_cmd=[{motor['i_cmd'][0]:+.2f}, {motor['i_cmd'][1]:+.2f}] A  "
                    f"tau_current=[{motor['tau_current'][0]:+.3f}, {motor['tau_current'][1]:+.3f}] Nm  "
                    f"tau_limit=[{motor['tau_speed_limit'][0]:+.3f}, {motor['tau_speed_limit'][1]:+.3f}] Nm  "
                    f"tau_actual=[{motor['tau_actual'][0]:+.3f}, {motor['tau_actual'][1]:+.3f}] Nm  "
                    f"rt_factor={(timestep * dt) / max(time.time() - wall_start, 1e-6):.2f}x"
                )
            sleep_time = dt - (time.time() - start)
            if args_cli.real_time and sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        if plotter is not None:
            plotter.close()
        logger.close(); env.close()


if __name__ == "__main__":
    main()  # type: ignore[reportCallIssue]
    simulation_app.close()
