# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
LQR, DDSM115, disturbance, and residual-policy diagnostic runner.

Purpose:
    Launch Isaac Sim for one-env diagnostics, including DDSM115 free-spin tests,
    suspended LQR sign checks, floor-contact LQR balancing, actuator/current
    disturbances, residual-policy evaluation, CSV logging, and live plotting.

Edit here when:
    You need to change diagnostic LQR behavior, frozen residual-policy
    evaluation, scripted disturbances, CSV metrics, or live plot signal groups.

Avoid changing here without also checking:
    residual_lqr_env.py for LQR/residual training parity, standup_env.py for the
    DDSM115 motor path, and downstream plotting scripts for CSV column names.
"""

from __future__ import annotations

# =============================================================================
# IMPORTS AND ISAAC APP LAUNCH
# =============================================================================

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
parser.add_argument("--control-decimation", type=int, default=20, help="Physics steps per action update.")
parser.add_argument("--solver-position-iters", type=int, default=8, help="PhysX articulation position solver iterations for this diagnostic.")
parser.add_argument("--solver-velocity-iters", type=int, default=4, help="PhysX articulation velocity solver iterations for this diagnostic.")
parser.add_argument("--max-depenetration-velocity", type=float, default=0.5, help="PhysX max depenetration velocity for this diagnostic.")
parser.add_argument("--base-height", type=float, default=0.50, help="Fixed base height above ground in meters.")
parser.add_argument(
    "--test-mode",
    choices=("free-spin", "lqr-model", "lqr-floor"),
    default="free-spin",
    help="Select constant-current free-spin, suspended LQR model diagnostic, or opt-in floor-contact LQR test.",
)
parser.add_argument("--motor-test-current", type=float, default=0.25, help="Desired current for both free-spinning wheels in A.")
parser.add_argument("--free-spin-test-time-s", type=float, default=0.0, help="Stop free-spin mode after this simulated duration; zero runs until interrupted.")
parser.add_argument("--left-motor-test-current", type=float, default=None, help="Optional left-wheel current override in A.")
parser.add_argument("--right-motor-test-current", type=float, default=None, help="Optional right-wheel current override in A.")
parser.add_argument(
    "--sign-diagnostic",
    action="store_true",
    help="In free-spin mode, command +current to left then right wheel and print raw/physical sign observations.",
)
parser.add_argument("--sign-diagnostic-current-a", type=float, default=0.20, help="Positive current used by --sign-diagnostic.")
parser.add_argument("--sign-diagnostic-phase-time-s", type=float, default=0.75, help="Seconds per left/coast/right sign diagnostic phase.")
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
    help="Initial model-state pitch for lqr-floor mode; magnitude must be at most 3 degrees. Zero is allowed.",
)
parser.add_argument("--floor-motors-disabled", action="store_true", help="Run the lqr-floor baseline with zero wheel current.")
parser.add_argument("--max-test-time-s", type=float, default=5.0, help="Stop lqr-floor diagnostics after this simulated duration.")
parser.add_argument("--floor-stop-pitch-deg", type=float, default=10.0, help="Stop lqr-floor diagnostics when |theta| exceeds this value.")
parser.add_argument("--floor-auto-stop-margin-deg", type=float, default=5.0, help="Automatically raise stop threshold this many degrees above the initial pitch if needed.")
parser.add_argument("--floor-settle-steps", type=int, default=10, help="Zero-torque physics steps to settle wheel contacts before LQR starts.")
parser.add_argument("--floor-contact-force-threshold", type=float, default=0.1, help="Minimum per-wheel contact force considered touching during floor startup checks.")
parser.add_argument(
    "--actuator-model",
    choices=("mujoco-torque", "ddsm115"),
    default="mujoco-torque",
    help="Wheel actuator model. Default matches MuJoCo direct torque for sim2sim diagnostics.",
)
parser.add_argument(
    "--lqr-output",
    choices=("torque", "force"),
    default="torque",
    help="Interpret fixed 4-state LQR gains as direct per-wheel torque or as force converted to DDSM115 current.",
)
parser.add_argument("--mujoco-torque-limit", type=float, default=4.0, help="Direct wheel torque limit in Nm for --actuator-model mujoco-torque.")
parser.add_argument("--mujoco-wheel-damping", type=float, default=0.2, help="Wheel joint damping in Nm*s/rad used to match MuJoCo XML.")
parser.add_argument(
    "--disturbance",
    choices=("none", "forward", "backward", "yaw-left", "yaw-right", "forward-yaw"),
    default="none",
    help="Optional scripted external wrench for lqr-floor disturbance tests.",
)
parser.add_argument("--disturbance-start-s", type=float, default=1.5, help="Sim time when the scripted disturbance starts.")
parser.add_argument("--disturbance-duration-s", type=float, default=0.20, help="Duration of the scripted disturbance.")
parser.add_argument("--disturbance-force-n", type=float, default=5.0, help="Forward/backward disturbance force magnitude in newtons.")
parser.add_argument("--disturbance-yaw-torque-nm", type=float, default=0.25, help="Yaw disturbance torque magnitude in Nm.")
parser.add_argument("--disturbance-body", type=str, default="Platform_Group", help="Robot body name receiving the external wrench.")
parser.add_argument(
    "--actuator-disturbance",
    choices=("none", "physical-forward", "step-forward"),
    default="none",
    help="Optional deterministic two-wheel current disturbance added after the LQR command in lqr-floor mode.",
)
parser.add_argument("--actuator-disturbance-start-s", type=float, default=1.5, help="Requested start time for the actuator disturbance.")
parser.add_argument("--actuator-disturbance-samples", type=int, default=2, help="Number of control samples to inject the actuator disturbance.")
parser.add_argument(
    "--actuator-disturbance-duration-s",
    type=float,
    default=0.0,
    help="Duration in seconds for step-forward actuator disturbance. Pulse-style disturbances use --actuator-disturbance-samples.",
)
parser.add_argument(
    "--actuator-disturbance-current-a",
    type=float,
    default=0.20,
    help="Physical-forward disturbance magnitude in A. Applied as left=+value, right=-value.",
)
parser.add_argument("--enable-residual-rl", action="store_true", help="Enable bounded PPO residual current on top of LQR.")
parser.add_argument("--residual-action-limit", type=float, default=0.5, help="Absolute residual current limit per wheel in A.")
parser.add_argument("--residual-policy", type=str, default="", help="Frozen TorchScript residual policy path for evaluation.")
parser.add_argument("--training-mode", action="store_true", help="Mark this run as residual-RL training mode for logs/config checks.")
parser.add_argument("--evaluation-mode", action="store_true", help="Mark this run as frozen-policy evaluation mode.")
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
from types import MethodType
from typing import Any

import gymnasium as gym
import numpy as np
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from isaaclab.envs import DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config
from TwoWheeledRobot.tasks.direct.twowheeledrobot.sim_params import (
    DDSM115_I_PEAK,
    DDSM115_KT,
    GROUND_DYNAMIC_FRICTION,
    GROUND_STATIC_FRICTION,
)


@dataclass(frozen=True)
class LqrPhysicalParams:
    body_mass_kg: float
    wheel_cart_mass_kg: float
    wheel_radius_m: float
    body_com_height_m: float
    body_pitch_inertia_kg_m2: float
    wheel_torque_constant_nm_per_a: float
    track_width_m: float
    body_yaw_inertia_kg_m2: float
    no_load_current_a: float = 0.25
    no_load_speed_rpm: float = 200.0
    gravity_m_s2: float = 9.81

    @property
    def wheel_internal_damping_nm_s_rad(self) -> float:
        omega_no_load_rad_s = self.no_load_speed_rpm * 2.0 * math.pi / 60.0
        return self.wheel_torque_constant_nm_per_a * self.no_load_current_a / omega_no_load_rad_s


# =============================================================================
# LQR PARAMETER DEFINITIONS
# =============================================================================

# ---------------------------------------------------------------------------
# USER LQR TUNING SECTION
# ---------------------------------------------------------------------------
# One controller design is used here: two mirrored, independent 4-state LQR
# controllers. Both wheels share this exact gain vector. State units are:
# [wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s].
#
# IMPORTANT: these editable gains are PER-WHEEL force gains. The standard
# cart-pendulum model in scripts/calculate_lqr_gains.py solves for total cart
# force first, then prints 0.5 * K_total as the paste-ready split-controller
# values below. Therefore each wheel-side LQR outputs its own physical
# wheel-ground force and there is no additional divide-by-two here.
K_POSITION_FORCE_PER_WHEEL = 23.8188
K_VELOCITY_FORCE_PER_WHEEL = 16.7186
K_PITCH_FORCE_PER_WHEEL = 77.3774
K_PITCH_RATE_FORCE_PER_WHEEL =4.5646

PER_WHEEL_FORCE_SCALE = 1.0
K_SPLIT_4_STATE_LQR_FORCE_PER_WHEEL = PER_WHEEL_FORCE_SCALE * np.array(
    [
        K_POSITION_FORCE_PER_WHEEL,
        K_VELOCITY_FORCE_PER_WHEEL,
        K_PITCH_FORCE_PER_WHEEL,
        K_PITCH_RATE_FORCE_PER_WHEEL,
    ],
    dtype=float,
)
K_SPLIT_4_STATE_LQR_FORCE_TOTAL_EQUIVALENT = 2.0 * K_SPLIT_4_STATE_LQR_FORCE_PER_WHEEL

# Raw wheel joint axes are mirrored in the USD. Convert raw encoder feedback to
# the physical convention before any controller sees it: positive means the
# wheel motion contributes to forward robot travel.
WHEEL_FEEDBACK_SIGN_VALUES = (-1.0, +1.0)

# The action accepted by StandupEnv is already physical forward-positive current.
# StandupEnv._pre_physics_step() applies the USD left-joint effort mirror:
#   effort_left = -torque_left, effort_right = +torque_right
# so lqr_control.py must not add another motor-axis mirror here.
WHEEL_COMMAND_SIGN_VALUES = (+1.0, +1.0)
WHEEL_EFFORT_SIGN_VALUES = (-1.0, +1.0)

# Projected-gravity pitch is opposite the positive pitch convention used by the
# current LQR gains. Keep the conversion explicit at the controller boundary so
# sensor logging remains unchanged.
LQR_PITCH_SENSOR_SIGN = -1.0
MUJOCO_LQR_PITCH_SENSOR_SIGN = +1.0

# Physical parameters retained for unit conversion and diagnostics. Yaw-related
# values are not controller states in this script; yaw is logged only.
LQR_PHYSICAL_PARAMS = LqrPhysicalParams(
    body_mass_kg=2.6,
    wheel_cart_mass_kg=1.53,
    wheel_radius_m=0.05035,
    body_com_height_m=0.14,
    body_pitch_inertia_kg_m2=0.0129596588636,
    wheel_torque_constant_nm_per_a=0.75,
    track_width_m=0.382999941707,
    body_yaw_inertia_kg_m2=0.0315051945189,
)
# ---------------------------------------------------------------------------
# END USER LQR TUNING SECTION
# ---------------------------------------------------------------------------

R_WHEEL = LQR_PHYSICAL_PARAMS.wheel_radius_m
MOTOR_TORQUE_CONSTANT_NM_PER_A = LQR_PHYSICAL_PARAMS.wheel_torque_constant_nm_per_a
FORCE_TO_CURRENT_A_PER_N = R_WHEEL / MOTOR_TORQUE_CONSTANT_NM_PER_A
CURRENT_TO_FORCE_N_PER_A = MOTOR_TORQUE_CONSTANT_NM_PER_A / R_WHEEL

# Keep the old current-space equivalent available only for equivalence diagnostics.
K_SPLIT_4_STATE_LQR_CURRENT_EQUIVALENT = K_SPLIT_4_STATE_LQR_FORCE_PER_WHEEL * FORCE_TO_CURRENT_A_PER_N


def active_lqr_pitch_sign() -> float:
    """Return the pitch sign convention for the selected LQR output model."""
    return MUJOCO_LQR_PITCH_SENSOR_SIGN if args_cli.lqr_output == "torque" else LQR_PITCH_SENSOR_SIGN


# =============================================================================
# LQR MODEL AND GAIN CALCULATION
# =============================================================================

def force_to_current_a(force_n: float) -> float:
    """Convert physical wheel-ground force to physical DDSM115 current."""
    return force_n * FORCE_TO_CURRENT_A_PER_N


def current_to_force_n(current_a: float) -> float:
    """Convert physical DDSM115 current to physical wheel-ground force."""
    return current_a * CURRENT_TO_FORCE_N_PER_A


def lqr_state_vector(
    wheel_position_m: float,
    wheel_velocity_m_s: float,
    pitch_rad: float,
    pitch_rate_rad_s: float,
) -> np.ndarray:
    return np.array(
        [wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s],
        dtype=float,
    )


def lqr_force_terms(
    wheel_position_m: float,
    wheel_velocity_m_s: float,
    pitch_rad: float,
    pitch_rate_rad_s: float,
) -> tuple[float, float, float, float]:
    """Return per-state force contributions before summing the LQR output."""
    return (
        -K_POSITION_FORCE_PER_WHEEL * wheel_position_m,
        -K_VELOCITY_FORCE_PER_WHEEL * wheel_velocity_m_s,
        -K_PITCH_FORCE_PER_WHEEL * pitch_rad,
        -K_PITCH_RATE_FORCE_PER_WHEEL * pitch_rate_rad_s,
    )


def compute_mujoco_style_lqr_torques(
    wheel_position_m: float,
    wheel_velocity_m_s: float,
    pitch_rad: float,
    pitch_rate_rad_s: float,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    """Return MuJoCo-style split wheel torques and equivalent currents.

    The MuJoCo diagnostic uses one common 4-state controller on average wheel
    path state, then applies tau_left=tau_right=-u_balance/2. Keep that path
    explicit so sim2sim tests do not silently use the force/current controller.
    """
    terms = lqr_force_terms(wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s)
    u_balance = sum(terms)
    torque = -0.5 * u_balance
    torques = (torque, torque)
    equivalent_currents = (torque / DDSM115_KT, torque / DDSM115_KT)
    return terms, torques, equivalent_currents


def compute_single_wheel_4_state_lqr_force(
    wheel_position_m: float,
    wheel_velocity_m_s: float,
    pitch_rad: float,
    pitch_rate_rad_s: float,
) -> float:
    """Return one wheel's physical 4-state LQR force in newtons.

    Inputs are one wheel path error in meters, one wheel velocity error in m/s,
    pitch in radians, and pitch rate in rad/s. This function intentionally does
    not know whether the wheel is left or right. Both wheels use the same force
    gain vector. Current conversion and mirrored command signs happen later.
    """
    return float(sum(lqr_force_terms(
        wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s
    )))


def compute_single_wheel_4_state_lqr_current_equivalent(
    wheel_position_m: float,
    wheel_velocity_m_s: float,
    pitch_rad: float,
    pitch_rate_rad_s: float,
) -> float:
    """Return the old current-space LQR output for force/current equivalence checks."""
    return float(-(K_SPLIT_4_STATE_LQR_CURRENT_EQUIVALENT @ lqr_state_vector(
        wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s
    )))


def compute_split_4_state_lqr_currents(
    state: RobotState,
    pitch_rad: float,
    pitch_rate_rad_s: float,
) -> tuple[float, float]:
    """Return left/right current commands from split force-space 4-state LQR."""
    _forces, _physical_currents, commanded, _max_diff = compute_split_4_state_lqr_outputs_for_values(
        left_wheel_position_m=state.wheel_position_rad[0] * R_WHEEL,
        left_wheel_velocity_m_s=state.wheel_velocity_rad_s[0] * R_WHEEL,
        right_wheel_position_m=state.wheel_position_rad[1] * R_WHEEL,
        right_wheel_velocity_m_s=state.wheel_velocity_rad_s[1] * R_WHEEL,
        pitch_rad=pitch_rad,
        pitch_rate_rad_s=pitch_rate_rad_s,
    )
    return commanded


def compute_split_4_state_lqr_outputs_for_values(
    left_wheel_position_m: float = 0.0,
    left_wheel_velocity_m_s: float = 0.0,
    right_wheel_position_m: float = 0.0,
    right_wheel_velocity_m_s: float = 0.0,
    pitch_rad: float = 0.0,
    pitch_rate_rad_s: float = 0.0,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float], float]:
    """Return force, physical current, command current, and old/new current diff."""
    left_force = compute_single_wheel_4_state_lqr_force(
        left_wheel_position_m, left_wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s
    )
    right_force = compute_single_wheel_4_state_lqr_force(
        right_wheel_position_m, right_wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s
    )
    physical_current = (
        force_to_current_a(left_force),
        force_to_current_a(right_force),
    )
    commanded = (
        WHEEL_COMMAND_SIGN_VALUES[0] * physical_current[0],
        WHEEL_COMMAND_SIGN_VALUES[1] * physical_current[1],
    )

    old_current = (
        compute_single_wheel_4_state_lqr_current_equivalent(
            left_wheel_position_m, left_wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s
        ),
        compute_single_wheel_4_state_lqr_current_equivalent(
            right_wheel_position_m, right_wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s
        ),
    )
    max_abs_diff = max(
        abs(old_current[0] - physical_current[0]),
        abs(old_current[1] - physical_current[1]),
    )
    return (left_force, right_force), physical_current, commanded, max_abs_diff


def compute_split_4_state_lqr_currents_for_values(
    left_wheel_position_m: float = 0.0,
    left_wheel_velocity_m_s: float = 0.0,
    right_wheel_position_m: float = 0.0,
    right_wheel_velocity_m_s: float = 0.0,
    pitch_rad: float = 0.0,
    pitch_rate_rad_s: float = 0.0,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return physical and mirrored command currents for diagnostic values."""
    _forces, physical_current, commanded, _max_diff = compute_split_4_state_lqr_outputs_for_values(
        left_wheel_position_m,
        left_wheel_velocity_m_s,
        right_wheel_position_m,
        right_wheel_velocity_m_s,
        pitch_rad,
        pitch_rate_rad_s,
    )
    return physical_current, commanded


@dataclass(frozen=True)
class RobotState:
    """State exposed to the user controller, in physical units."""

    time_s: float
    projected_gravity_body: tuple[float, float, float]
    angular_velocity_body_rad_s: tuple[float, float, float]
    angular_velocity_world_rad_s: tuple[float, float, float]
    root_position_world_m: tuple[float, float, float]
    root_orientation_world_quat_wxyz: tuple[float, float, float, float]
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


@dataclass
class DisturbanceState:
    body_ids: torch.Tensor | None = None
    body_names: list[str] | None = None
    active: bool = False
    force_world_n: tuple[float, float, float] = (0.0, 0.0, 0.0)
    torque_world_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)


DISTURBANCE_STATE = DisturbanceState()


@dataclass
class ActuatorCommandDebug:
    sample_index: int = 0
    command_time_s: float = 0.0
    active: bool = False
    active_sample_index: int = -1
    start_sample_index: int = -1
    stop_sample_index: int = -1
    lqr_state_position_m: tuple[float, float] = (0.0, 0.0)
    lqr_state_velocity_m_s: tuple[float, float] = (0.0, 0.0)
    lqr_state_pitch_rad: float = 0.0
    lqr_state_pitch_rate_rad_s: float = 0.0
    lqr_force_position_n: tuple[float, float] = (0.0, 0.0)
    lqr_force_velocity_n: tuple[float, float] = (0.0, 0.0)
    lqr_force_pitch_n: tuple[float, float] = (0.0, 0.0)
    lqr_force_pitch_rate_n: tuple[float, float] = (0.0, 0.0)
    lqr_force_physical_n: tuple[float, float] = (0.0, 0.0)
    lqr_current_physical_a: tuple[float, float] = (0.0, 0.0)
    lqr_current_equivalence_error_a: float = 0.0
    lqr_current_a: tuple[float, float] = (0.0, 0.0)
    rl_current_a: tuple[float, float] = (0.0, 0.0)
    disturbance_current_a: tuple[float, float] = (0.0, 0.0)
    commanded_current_a: tuple[float, float] = (0.0, 0.0)
    reward: float = 0.0
    episode_reward: float = 0.0


ACTUATOR_COMMAND_DEBUG = ActuatorCommandDebug()
RESIDUAL_POLICY: torch.jit.ScriptModule | torch.nn.Module | None = None
PREVIOUS_RL_CURRENT_A = [0.0, 0.0]
RESIDUAL_EPISODE_REWARD = 0.0


# =============================================================================
# RESIDUAL POLICY LOADING
# =============================================================================

def load_residual_policy() -> torch.jit.ScriptModule | torch.nn.Module | None:
    """Load a frozen TorchScript residual policy on first use."""
    global RESIDUAL_POLICY
    if not args_cli.enable_residual_rl or not args_cli.residual_policy:
        return None
    if RESIDUAL_POLICY is None:
        policy_path = Path(args_cli.residual_policy)
        if not policy_path.is_file():
            raise FileNotFoundError(f"Residual policy not found: {policy_path}")
        try:
            RESIDUAL_POLICY = torch.jit.load(str(policy_path), map_location="cpu")
        except RuntimeError as exc:
            exported_hint = policy_path.parent / "exported" / "policy.pt"
            hint = (
                f" Use the exported TorchScript policy instead: {exported_hint}"
                if exported_hint.is_file()
                else " Export the checkpoint first with scripts/rsl_rl/play.py, then use <run>/exported/policy.pt."
            )
            raise RuntimeError(
                f"Residual policy must be a TorchScript exported policy.pt, not an RSL-RL training checkpoint: "
                f"{policy_path}.{hint}"
            ) from exc
        RESIDUAL_POLICY.eval()
        print(f"[RESIDUAL RL]: loaded frozen policy {policy_path}")
    return RESIDUAL_POLICY


def residual_observation(
    state: RobotState,
    theta: float,
    theta_dot: float,
    base_position: float,
    base_velocity: float,
) -> torch.Tensor:
    """Build the 8D residual PPO observation for frozen-policy evaluation.

    Units and order must match residual_lqr_env.py::_get_observations(): pitch
    rad, pitch rate rad/s, base position m, base velocity m/s, normalized wheel
    velocities, and previous residual currents normalized by the residual limit.
    """
    return torch.tensor(
        [
            theta,
            theta_dot,
            base_position,
            base_velocity,
            state.wheel_velocity_rad_s[0] / LQR_PHYSICAL_PARAMS.no_load_speed_rpm * 60.0 / (2.0 * math.pi),
            state.wheel_velocity_rad_s[1] / LQR_PHYSICAL_PARAMS.no_load_speed_rpm * 60.0 / (2.0 * math.pi),
            PREVIOUS_RL_CURRENT_A[0] / max(args_cli.residual_action_limit, 1.0e-6),
            PREVIOUS_RL_CURRENT_A[1] / max(args_cli.residual_action_limit, 1.0e-6),
        ],
        dtype=torch.float32,
    ).unsqueeze(0)


def residual_current_from_policy(
    state: RobotState,
    theta: float,
    theta_dot: float,
    base_position: float,
    base_velocity: float,
) -> tuple[float, float]:
    """Return bounded residual current in amperes.

    The TorchScript policy action is clamped to [-1, 1] and scaled by
    --residual-action-limit. This mirrors residual_lqr_env.py action scaling for
    frozen-policy evaluation.
    """
    if not args_cli.enable_residual_rl:
        return (0.0, 0.0)
    policy = load_residual_policy()
    if policy is None:
        return (0.0, 0.0)
    obs = residual_observation(state, theta, theta_dot, base_position, base_velocity)
    with torch.inference_mode():
        raw_action = policy(obs)
    action = torch.as_tensor(raw_action, dtype=torch.float32).reshape(-1)[:2].clamp(-1.0, 1.0)
    residual = action * float(args_cli.residual_action_limit)
    return (float(residual[0]), float(residual[1]))


def residual_reward(theta: float, theta_dot: float, position: float, rl_current: tuple[float, float]) -> float:
    return (
        -(theta ** 2)
        - 0.1 * (theta_dot ** 2)
        - 0.05 * (position ** 2)
        - 0.01 * (rl_current[0] ** 2 + rl_current[1] ** 2)
    )


# =============================================================================
# DISTURBANCE GENERATION
# =============================================================================

# ---------------------------------------------------------------------------
# USER CONTROL SECTION
# ---------------------------------------------------------------------------
def actuator_disturbance_current(control_sample: int) -> tuple[bool, tuple[float, float], int]:
    """Return deterministic actuator-current disturbance for one control sample.

    The returned current is in amperes and is added after LQR and residual
    currents. Sweep scripts reach pulse/step behavior indirectly through this
    function.
    """
    if args_cli.actuator_disturbance == "none":
        return False, (0.0, 0.0), -1
    if args_cli.test_mode != "lqr-floor":
        return False, (0.0, 0.0), -1
    start = ACTUATOR_COMMAND_DEBUG.start_sample_index
    stop = ACTUATOR_COMMAND_DEBUG.stop_sample_index
    active = start <= control_sample < stop
    if not active:
        return False, (0.0, 0.0), -1
    magnitude = args_cli.actuator_disturbance_current_a
    if args_cli.actuator_disturbance in ("physical-forward", "step-forward"):
        # Currents are physical forward-positive here. StandupEnv applies the
        # left USD joint-axis mirror when converting current to joint effort, so
        # a forward actuator disturbance uses the same sign on both wheels.
        return True, (magnitude, magnitude), control_sample - start
    raise ValueError(f"Unsupported actuator disturbance '{args_cli.actuator_disturbance}'.")


# =============================================================================
# ACTION COMPUTATION
# =============================================================================

def compute_action(state: RobotState, control_sample: int) -> RobotAction:
    """Return physical joint/current commands for the selected diagnostic.

    Args:
        state: Robot state in physical units from read_state().
        control_sample: Integer controller sample index at the environment
            control rate.

    Returns:
        RobotAction with CyberGear target angles in radians and left/right wheel
        current commands in amperes.
    """
    global RESIDUAL_EPISODE_REWARD
    lqr_state_position = (0.0, 0.0)
    lqr_state_velocity = (0.0, 0.0)
    lqr_state_pitch = 0.0
    lqr_state_pitch_rate = 0.0
    lqr_force_position = (0.0, 0.0)
    lqr_force_velocity = (0.0, 0.0)
    lqr_force_pitch = (0.0, 0.0)
    lqr_force_pitch_rate = (0.0, 0.0)
    lqr_force_physical = (0.0, 0.0)
    lqr_current_physical = (0.0, 0.0)
    lqr_current_equivalence_error = 0.0
    lqr_current = (0.0, 0.0)
    rl_current = (0.0, 0.0)
    disturbance_current = (0.0, 0.0)
    disturbance_active = False
    disturbance_active_sample = -1
    reward = 0.0
    if args_cli.test_mode == "free-spin":
        if args_cli.sign_diagnostic:
            phase_time = max(args_cli.sign_diagnostic_phase_time_s, 1.0e-6)
            phase = int(state.time_s // phase_time)
            current = args_cli.sign_diagnostic_current_a
            if phase == 0:
                wheel_current = (current, 0.0)
            elif phase == 1:
                wheel_current = (0.0, 0.0)
            elif phase == 2:
                wheel_current = (0.0, current)
            else:
                wheel_current = (0.0, 0.0)
        else:
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
            lqr_state_position = (0.0, 0.0)
            lqr_state_velocity = (0.0, 0.0)
            pitch_sign = active_lqr_pitch_sign()
            lqr_state_pitch = pitch_sign * theta
            lqr_state_pitch_rate = pitch_sign * theta_dot
            if args_cli.lqr_output == "torque":
                terms, lqr_current, lqr_current_physical = compute_mujoco_style_lqr_torques(
                    0.0, 0.0, lqr_state_pitch, lqr_state_pitch_rate
                )
                lqr_force_physical = lqr_current
                left_terms = terms
                right_terms = terms
            else:
                lqr_force_physical, lqr_current_physical, lqr_current, lqr_current_equivalence_error = (
                    compute_split_4_state_lqr_outputs_for_values(
                        pitch_rad=lqr_state_pitch,
                        pitch_rate_rad_s=lqr_state_pitch_rate,
                    )
                )
                left_terms = lqr_force_terms(0.0, 0.0, lqr_state_pitch, lqr_state_pitch_rate)
                right_terms = left_terms
            lqr_force_position = (left_terms[0], right_terms[0])
            lqr_force_velocity = (left_terms[1], right_terms[1])
            lqr_force_pitch = (left_terms[2], right_terms[2])
            lqr_force_pitch_rate = (left_terms[3], right_terms[3])
        else:
            theta = pitch_from_projected_gravity(state.projected_gravity_body)
            theta_dot = -state.angular_velocity_body_rad_s[0]
            wheel_position_m = 0.5 * sum(state.wheel_position_rad) * R_WHEEL
            if args_cli.floor_motors_disabled:
                lqr_state_position = (0.0, 0.0)
                lqr_state_velocity = (0.0, 0.0)
                lqr_state_pitch = 0.0
                lqr_state_pitch_rate = 0.0
                lqr_force_position = (0.0, 0.0)
                lqr_force_velocity = (0.0, 0.0)
                lqr_force_pitch = (0.0, 0.0)
                lqr_force_pitch_rate = (0.0, 0.0)
                lqr_force_physical = (0.0, 0.0)
                lqr_current_physical = (0.0, 0.0)
                lqr_current_equivalence_error = 0.0
                lqr_current = (0.0, 0.0)
            else:
                if args_cli.lqr_output == "torque":
                    lqr_state_position = (wheel_position_m, wheel_position_m)
                    lqr_state_velocity = (0.5 * sum(state.wheel_velocity_rad_s) * R_WHEEL,) * 2
                else:
                    lqr_state_position = (state.wheel_position_rad[0] * R_WHEEL, state.wheel_position_rad[1] * R_WHEEL)
                    lqr_state_velocity = (state.wheel_velocity_rad_s[0] * R_WHEEL, state.wheel_velocity_rad_s[1] * R_WHEEL)
                pitch_sign = active_lqr_pitch_sign()
                lqr_state_pitch = pitch_sign * theta
                lqr_state_pitch_rate = pitch_sign * theta_dot
                if args_cli.lqr_output == "torque":
                    terms, lqr_current, lqr_current_physical = compute_mujoco_style_lqr_torques(
                        lqr_state_position[0], lqr_state_velocity[0], lqr_state_pitch, lqr_state_pitch_rate
                    )
                    lqr_force_physical = lqr_current
                    left_terms = terms
                    right_terms = terms
                else:
                    lqr_force_physical, lqr_current_physical, lqr_current, lqr_current_equivalence_error = (
                        compute_split_4_state_lqr_outputs_for_values(
                            left_wheel_position_m=lqr_state_position[0],
                            left_wheel_velocity_m_s=lqr_state_velocity[0],
                            right_wheel_position_m=lqr_state_position[1],
                            right_wheel_velocity_m_s=lqr_state_velocity[1],
                            pitch_rad=lqr_state_pitch,
                            pitch_rate_rad_s=lqr_state_pitch_rate,
                        )
                    )
                    left_terms = lqr_force_terms(
                        lqr_state_position[0], lqr_state_velocity[0], lqr_state_pitch, lqr_state_pitch_rate
                    )
                    right_terms = lqr_force_terms(
                        lqr_state_position[1], lqr_state_velocity[1], lqr_state_pitch, lqr_state_pitch_rate
                    )
                lqr_force_position = (left_terms[0], right_terms[0])
                lqr_force_velocity = (left_terms[1], right_terms[1])
                lqr_force_pitch = (left_terms[2], right_terms[2])
                lqr_force_pitch_rate = (left_terms[3], right_terms[3])

        # Residual RL is intentionally disabled in this split 4-state LQR path.
        # The diagnostic CLI flags remain accepted, but they do not alter LQR
        # current commands for this controller.
        rl_current = (0.0, 0.0)
        disturbance_active, disturbance_current, disturbance_active_sample = actuator_disturbance_current(control_sample)
        lqr_command = lqr_current
        if args_cli.lqr_output == "torque" and args_cli.actuator_model == "mujoco-torque":
            disturbance_current = (
                disturbance_current[0] * DDSM115_KT,
                disturbance_current[1] * DDSM115_KT,
            )
        elif args_cli.lqr_output == "torque":
            lqr_command = lqr_current_physical
        wheel_current = (
            lqr_command[0] + disturbance_current[0],
            lqr_command[1] + disturbance_current[1],
        )
        reward = residual_reward(theta, theta_dot, wheel_position_m, rl_current)
        RESIDUAL_EPISODE_REWARD += reward

    PREVIOUS_RL_CURRENT_A[0], PREVIOUS_RL_CURRENT_A[1] = rl_current
    ACTUATOR_COMMAND_DEBUG.sample_index = control_sample
    ACTUATOR_COMMAND_DEBUG.command_time_s = state.time_s
    ACTUATOR_COMMAND_DEBUG.active = disturbance_active
    ACTUATOR_COMMAND_DEBUG.active_sample_index = disturbance_active_sample
    ACTUATOR_COMMAND_DEBUG.lqr_state_position_m = lqr_state_position
    ACTUATOR_COMMAND_DEBUG.lqr_state_velocity_m_s = lqr_state_velocity
    ACTUATOR_COMMAND_DEBUG.lqr_state_pitch_rad = lqr_state_pitch
    ACTUATOR_COMMAND_DEBUG.lqr_state_pitch_rate_rad_s = lqr_state_pitch_rate
    ACTUATOR_COMMAND_DEBUG.lqr_force_position_n = lqr_force_position
    ACTUATOR_COMMAND_DEBUG.lqr_force_velocity_n = lqr_force_velocity
    ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_n = lqr_force_pitch
    ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_rate_n = lqr_force_pitch_rate
    ACTUATOR_COMMAND_DEBUG.lqr_force_physical_n = lqr_force_physical
    ACTUATOR_COMMAND_DEBUG.lqr_current_physical_a = lqr_current_physical
    ACTUATOR_COMMAND_DEBUG.lqr_current_equivalence_error_a = lqr_current_equivalence_error
    ACTUATOR_COMMAND_DEBUG.lqr_current_a = lqr_current
    ACTUATOR_COMMAND_DEBUG.rl_current_a = rl_current
    ACTUATOR_COMMAND_DEBUG.disturbance_current_a = disturbance_current
    ACTUATOR_COMMAND_DEBUG.commanded_current_a = wheel_current
    ACTUATOR_COMMAND_DEBUG.reward = reward
    ACTUATOR_COMMAND_DEBUG.episode_reward = RESIDUAL_EPISODE_REWARD

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


def yaw_from_quat_wxyz(quat_wxyz: tuple[float, float, float, float]) -> float:
    """Return world-frame yaw from an Isaac Lab wxyz quaternion."""
    w, x, y, z = quat_wxyz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle_rad(value: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(value), math.cos(value))


def split_4_state_currents_for_synthetic_pitch(pitch_deg: float) -> tuple[float, float]:
    """Return max command currents for a zero-path synthetic pitch test."""
    _physical, commanded = compute_split_4_state_lqr_currents_for_values(
        pitch_rad=active_lqr_pitch_sign() * math.radians(pitch_deg),
        pitch_rate_rad_s=0.0,
    )
    return commanded


def lqr_current_for_pitch_deg(pitch_deg: float) -> float:
    """Return the max split 4-state command current for synthetic pitch."""
    left_current, right_current = split_4_state_currents_for_synthetic_pitch(pitch_deg)
    return max(abs(left_current), abs(right_current))


def print_lqr_diagnostic() -> None:
    """Print split 4-state LQR configuration and basic sign diagnostics."""
    params = LQR_PHYSICAL_PARAMS
    print("[LQR]: controller=split 4-state LQR")
    print("[LQR]: state per wheel = [wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s]")
    print("[LQR]: yaw is diagnostic only and is not a controller state")
    print("[LQR]: K_SPLIT_4_STATE_LQR_FORCE_PER_WHEEL =", K_SPLIT_4_STATE_LQR_FORCE_PER_WHEEL)
    print("[LQR]: K_SPLIT_4_STATE_LQR_FORCE_TOTAL_EQUIVALENT =", K_SPLIT_4_STATE_LQR_FORCE_TOTAL_EQUIVALENT)
    print(f"[LQR]: PER_WHEEL_FORCE_SCALE={PER_WHEEL_FORCE_SCALE:.6g}  # already per-wheel, no extra /2")
    print("[LQR]: K_SPLIT_4_STATE_LQR_CURRENT_EQUIVALENT =", K_SPLIT_4_STATE_LQR_CURRENT_EQUIVALENT)
    print(f"[LQR]: FORCE_TO_CURRENT_A_PER_N={FORCE_TO_CURRENT_A_PER_N:.12g}")
    print(f"[LQR]: CURRENT_TO_FORCE_N_PER_A={CURRENT_TO_FORCE_N_PER_A:.12g}")
    print(f"[LQR]: WHEEL_FEEDBACK_SIGN raw->physical={WHEEL_FEEDBACK_SIGN_VALUES}")
    print(f"[LQR]: WHEEL_COMMAND_SIGN physical->StandupEnv action current={WHEEL_COMMAND_SIGN_VALUES}")
    print(f"[LQR]: WHEEL_EFFORT_SIGN physical torque->USD joint effort={WHEEL_EFFORT_SIGN_VALUES}")
    print(f"[LQR]: LQR_PITCH_SENSOR_SIGN={LQR_PITCH_SENSOR_SIGN:+.1f}")
    print(f"[LQR]: MUJOCO_LQR_PITCH_SENSOR_SIGN={MUJOCO_LQR_PITCH_SENSOR_SIGN:+.1f}")
    print(f"[LQR]: active_lqr_pitch_sign={active_lqr_pitch_sign():+.1f}")
    print("[LQR]: StandupEnv applies the left USD joint mirror when converting current to effort")
    print("[LQR]: physical parameters retained for diagnostics/unit conversion:")
    print(f"  body_mass_kg={params.body_mass_kg:.6g}")
    print(f"  wheel_cart_mass_kg={params.wheel_cart_mass_kg:.6g}")
    print(f"  wheel_radius_m={params.wheel_radius_m:.6g}")
    print(f"  body_com_height_m={params.body_com_height_m:.6g}")
    print(f"  body_pitch_inertia_kg_m2={params.body_pitch_inertia_kg_m2:.12g}")
    print(f"  wheel_torque_constant_nm_per_a={params.wheel_torque_constant_nm_per_a:.6g}")
    print(f"  track_width_m={params.track_width_m:.12g}  # logged/diagnostic only")
    print(f"  body_yaw_inertia_kg_m2={params.body_yaw_inertia_kg_m2:.12g}  # logged/diagnostic only")

    pitch_force, pitch_physical_current, pitch_commanded_current, pitch_equivalence_error = (
        compute_split_4_state_lqr_outputs_for_values(
            pitch_rad=active_lqr_pitch_sign() * math.radians(1.0),
            pitch_rate_rad_s=0.0,
        )
    )
    print(
        "[SIGN TEST] pitch=+1.0 deg physical force -> "
        f"left={pitch_force[0]:+.6f} N right={pitch_force[1]:+.6f} N"
    )
    print(
        "[SIGN TEST] pitch=+1.0 deg physical current before command sign -> "
        f"left={pitch_physical_current[0]:+.6f} A right={pitch_physical_current[1]:+.6f} A"
    )
    print(
        "[SIGN TEST] pitch=+1.0 deg current sent to StandupEnv -> "
        f"left={pitch_commanded_current[0]:+.6f} A right={pitch_commanded_current[1]:+.6f} A"
    )
    print(f"[EQUIVALENCE TEST] pitch=+1.0 deg max_abs(i_old - i_new)={pitch_equivalence_error:.12g} A")
    if pitch_equivalence_error >= 1.0e-5:
        print("[EQUIVALENCE TEST] WARNING: force-space and current-space LQR outputs differ.")
    if abs(pitch_physical_current[0]) < 1.0e-12 or abs(pitch_physical_current[1]) < 1.0e-12:
        print("[SIGN TEST] WARNING: pitch-only physical currents are near zero.")
    if not math.isclose(pitch_physical_current[0], pitch_physical_current[1], rel_tol=1.0e-9, abs_tol=1.0e-12):
        print("[SIGN TEST] WARNING: pitch-only physical currents are not equal.")

    path_force, path_physical_current, path_commanded, path_equivalence_error = (
        compute_split_4_state_lqr_outputs_for_values(
            left_wheel_position_m=0.01,
            right_wheel_position_m=0.0,
            pitch_rad=0.0,
            pitch_rate_rad_s=0.0,
        )
    )
    print(
        "[PATH TEST] left path=+0.01 m, right path=0 physical force -> "
        f"left={path_force[0]:+.6f} N right={path_force[1]:+.6f} N"
    )
    print(
        "[PATH TEST] left path=+0.01 m, right path=0 physical current before command sign -> "
        f"left={path_physical_current[0]:+.6f} A right={path_physical_current[1]:+.6f} A"
    )
    print(
        "[PATH TEST] left path=+0.01 m, right path=0 current sent to StandupEnv -> "
        f"left={path_commanded[0]:+.6f} A right={path_commanded[1]:+.6f} A"
    )
    print(f"[EQUIVALENCE TEST] path max_abs(i_old - i_new)={path_equivalence_error:.12g} A")
    if path_equivalence_error >= 1.0e-5:
        print("[EQUIVALENCE TEST] WARNING: force-space and current-space LQR outputs differ.")
    if math.isclose(path_physical_current[0], path_physical_current[1], rel_tol=1.0e-9, abs_tol=1.0e-12):
        print("[PATH TEST] NOTE: physical currents are equal because K_POSITION_FORCE_PER_WHEEL is currently zero.")
        print("[PATH TEST] Nonzero K_POSITION_FORCE_PER_WHEEL would let split wheel path errors create differential correction.")


# =============================================================================
# STATE READING
# =============================================================================

def read_state(env: Any, time_s: float) -> RobotState:
    """Read env_0 and convert the mirrored left wheel axis to forward-positive."""
    data = env.robot.data
    raw_pos = data.joint_pos[0, env._wheel_ids]
    raw_vel = data.joint_vel[0, env._wheel_ids]
    wheel_sign = torch.tensor(WHEEL_FEEDBACK_SIGN_VALUES, device=env.device)
    return RobotState(
        time_s=time_s,
        projected_gravity_body=_tuple(env.bno080.data.projected_gravity_b[0]),
        angular_velocity_body_rad_s=_tuple(env.bno080.data.ang_vel_b[0]),
        angular_velocity_world_rad_s=_tuple(data.root_ang_vel_w[0]),
        root_position_world_m=_tuple(data.root_pos_w[0]),
        root_orientation_world_quat_wxyz=_tuple(data.root_quat_w[0]),
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


def install_mujoco_torque_actuator(env: Any) -> None:
    """Patch this diagnostic env to interpret wheel actions as direct torques."""

    def _pre_physics_step_mujoco_torque(self: Any, actions: torch.Tensor) -> None:
        actions = actions.clamp(-1.0, 1.0)
        self._enforce_cybergear_joint_state_limits()

        self._prev_actions = self._cur_actions.clone()
        self._cur_actions = actions.clone()

        act01 = 0.5 * (actions[:, 0:4] + 1.0)
        cg_targets = self._cg_joint_lo + act01 * (self._cg_joint_hi - self._cg_joint_lo)
        self.robot.set_joint_position_target(cg_targets, joint_ids=self._cg_ids)

        torque_limit = float(self.cfg.wheel_current_max)
        torque_des = actions[:, 4:6] * torque_limit
        torque_cmd = torque_des.clamp(-torque_limit, torque_limit)
        current_equivalent = torque_cmd / DDSM115_KT

        self._wheel_i_des = current_equivalent
        self._wheel_i_cmd = current_equivalent
        self._wheel_tau_current = torque_cmd
        self._wheel_velocity_raw = self.robot.data.joint_vel[:, self._wheel_ids].clone()
        self._wheel_velocity_used = self._wheel_velocity_raw.clone()
        self._wheel_omega_for_limiter = self._wheel_velocity_used.abs()
        self._wheel_tau_speed_limit = torch.full_like(torque_cmd, torque_limit)
        self._wheel_torque_cmd = torque_cmd

        self._efforts_buf[:, 0] = -self._wheel_torque_cmd[:, 0]
        self._efforts_buf[:, 1] = self._wheel_torque_cmd[:, 1]
        self.robot.set_joint_effort_target(self._efforts_buf, joint_ids=self._wheel_ids)

    env._pre_physics_step = MethodType(_pre_physics_step_mujoco_torque, env)


def read_motor_debug(env: Any) -> dict[str, tuple[float, float]]:
    """Read the shared DDSM115 model stages for environment 0."""
    return {
        "i_des": _tuple(env._wheel_i_des[0]),
        "i_cmd": _tuple(env._wheel_i_cmd[0]),
        "tau_current": _tuple(env._wheel_tau_current[0]),
        "wheel_velocity_raw": _tuple(env._wheel_velocity_raw[0]),
        "wheel_velocity_used": _tuple(env._wheel_velocity_used[0]),
        "omega_for_limiter": _tuple(env._wheel_omega_for_limiter[0]),
        "tau_speed_limit": _tuple(env._wheel_tau_speed_limit[0]),
        "tau_actual": _tuple(env._wheel_torque_cmd[0]),
        "applied_effort": _tuple(env._efforts_buf[0]),
    }


def read_wheel_contact_forces(env: Any) -> tuple[float, float]:
    """Return left/right wheel net contact-force magnitudes in newtons."""
    forces = env.wheel_contacts.data.net_forces_w[0, env._wheel_contact_ids]
    return tuple(float(v) for v in torch.linalg.vector_norm(forces, dim=-1).detach().cpu().tolist())


def settle_floor_contacts(env: Any) -> tuple[int, tuple[float, float]]:
    """Run zero-torque startup steps until both wheel contacts are initialized."""
    if args_cli.test_mode != "lqr-floor" or args_cli.floor_settle_steps <= 0:
        return 0, read_wheel_contact_forces(env) if args_cli.test_mode == "lqr-floor" else (0.0, 0.0)

    zero_action = action_to_env(env, RobotAction(
        cg_joint_position_rad=(0.0, 0.0, 0.0, 0.0),
        wheel_current_a=(0.0, 0.0),
    ))
    contact_forces = read_wheel_contact_forces(env)
    threshold = args_cli.floor_contact_force_threshold
    settled_steps = 0
    with torch.inference_mode():
        for step in range(args_cli.floor_settle_steps):
            if contact_forces[0] > threshold and contact_forces[1] > threshold:
                break
            env.step(zero_action)
            settled_steps = step + 1
            contact_forces = read_wheel_contact_forces(env)
    return settled_steps, contact_forces


def resolve_disturbance_body(env: Any) -> None:
    """Resolve the single body receiving the scripted external wrench."""
    if args_cli.disturbance == "none":
        return
    body_ids, body_names = env.robot.find_bodies(args_cli.disturbance_body, preserve_order=True)
    if not body_ids:
        available = ", ".join(env.robot.body_names)
        raise RuntimeError(
            f"Disturbance body '{args_cli.disturbance_body}' not found. "
            f"Available body names: {available}"
        )
    if len(body_ids) != 1:
        raise RuntimeError(f"Disturbance body pattern '{args_cli.disturbance_body}' matched multiple bodies: {body_names}")
    DISTURBANCE_STATE.body_ids = torch.tensor(body_ids, device=env.device, dtype=torch.long)
    DISTURBANCE_STATE.body_names = body_names


def disturbance_wrench(time_s: float) -> tuple[bool, tuple[float, float, float], tuple[float, float, float]]:
    """Return active flag plus world-frame force and torque for the configured disturbance."""
    if args_cli.disturbance == "none":
        return False, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    start = args_cli.disturbance_start_s
    end = start + args_cli.disturbance_duration_s
    active = start <= time_s < end
    if not active:
        return False, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    forward = (math.cos(YAW_REFERENCE_RAD), math.sin(YAW_REFERENCE_RAD), 0.0)
    force_scale = 0.0
    torque_z = 0.0
    if args_cli.disturbance in ("forward", "forward-yaw"):
        force_scale = args_cli.disturbance_force_n
    elif args_cli.disturbance == "backward":
        force_scale = -args_cli.disturbance_force_n
    if args_cli.disturbance in ("yaw-left", "forward-yaw"):
        torque_z = args_cli.disturbance_yaw_torque_nm
    elif args_cli.disturbance == "yaw-right":
        torque_z = -args_cli.disturbance_yaw_torque_nm
    force = (force_scale * forward[0], force_scale * forward[1], 0.0)
    torque = (0.0, 0.0, torque_z)
    return True, force, torque


def apply_disturbance(env: Any, time_s: float) -> None:
    """Apply or clear the configured scripted external wrench."""
    if args_cli.disturbance == "none":
        DISTURBANCE_STATE.active = False
        DISTURBANCE_STATE.force_world_n = (0.0, 0.0, 0.0)
        DISTURBANCE_STATE.torque_world_nm = (0.0, 0.0, 0.0)
        return
    if DISTURBANCE_STATE.body_ids is None:
        raise RuntimeError("Disturbance body was not resolved before applying disturbance.")

    active, force, torque = disturbance_wrench(time_s)
    forces = torch.tensor([[force]], device=env.device, dtype=torch.float32)
    torques = torch.tensor([[torque]], device=env.device, dtype=torch.float32)
    env.robot.permanent_wrench_composer.set_forces_and_torques(
        forces=forces,
        torques=torques,
        body_ids=DISTURBANCE_STATE.body_ids,
        is_global=True,
    )

    if active and not DISTURBANCE_STATE.active:
        print(
            "[DISTURBANCE]: active "
            f"mode={args_cli.disturbance} time={time_s:.3f} s "
            f"force_world={force} N torque_world={torque} Nm"
        )
    if not active and DISTURBANCE_STATE.active:
        print(f"[DISTURBANCE]: cleared time={time_s:.3f} s")
    DISTURBANCE_STATE.active = active
    DISTURBANCE_STATE.force_world_n = force
    DISTURBANCE_STATE.torque_world_nm = torque


def sample_to_row(state: RobotState, motor: dict[str, tuple[float, float]], contact_forces: tuple[float, float]) -> dict[str, float]:
    """Flatten state/action/motor debug values into CSV metric columns.

    Column names are consumed by sweep, repeatability, step-benchmark, and plot
    scripts. Add aliases instead of renaming existing columns unless all
    downstream readers are updated together.
    """
    theta = pitch_from_projected_gravity(state.projected_gravity_body)
    theta_dot = -state.angular_velocity_body_rad_s[0]
    position = 0.5 * sum(state.wheel_position_rad) * R_WHEEL
    velocity = 0.5 * sum(state.wheel_velocity_rad_s) * R_WHEEL
    # Yaw is logged as a diagnostic only. It is not a state in the split
    # 4-state LQR controller used by compute_action().
    base_yaw = yaw_from_quat_wxyz(state.root_orientation_world_quat_wxyz)
    base_yaw_rate = state.angular_velocity_world_rad_s[2]
    yaw = wrap_angle_rad(base_yaw - YAW_REFERENCE_RAD)
    yaw_rate = base_yaw_rate
    left_wheel_rpm = state.wheel_velocity_rad_s[0] * 60.0 / (2.0 * math.pi)
    right_wheel_rpm = state.wheel_velocity_rad_s[1] * 60.0 / (2.0 * math.pi)
    rpm_diff = right_wheel_rpm - left_wheel_rpm
    current_diff = motor["i_cmd"][1] - motor["i_cmd"][0]
    torque_diff = motor["tau_actual"][1] - motor["tau_actual"][0]
    contact_force_diff = contact_forces[1] - contact_forces[0]
    return {
        "time": state.time_s,
        "theta": theta,
        "theta_dot": theta_dot,
        "position": position,
        "velocity": velocity,
        "base_yaw": base_yaw,
        "base_yaw_rate": base_yaw_rate,
        "yaw": yaw,
        "yaw_rate": yaw_rate,
        "disturbance_active": float(DISTURBANCE_STATE.active),
        "disturbance_force_x_n": DISTURBANCE_STATE.force_world_n[0],
        "disturbance_force_y_n": DISTURBANCE_STATE.force_world_n[1],
        "disturbance_force_z_n": DISTURBANCE_STATE.force_world_n[2],
        "disturbance_torque_z_nm": DISTURBANCE_STATE.torque_world_nm[2],
        "actuator_disturbance_active": float(ACTUATOR_COMMAND_DEBUG.active),
        "actuator_disturbance_sample": float(ACTUATOR_COMMAND_DEBUG.active_sample_index),
        "actuator_command_sample": float(ACTUATOR_COMMAND_DEBUG.sample_index),
        "actuator_command_time_s": ACTUATOR_COMMAND_DEBUG.command_time_s,
        "wheel_feedback_sign_left": WHEEL_FEEDBACK_SIGN_VALUES[0],
        "wheel_feedback_sign_right": WHEEL_FEEDBACK_SIGN_VALUES[1],
        "wheel_command_sign_left": WHEEL_COMMAND_SIGN_VALUES[0],
        "wheel_command_sign_right": WHEEL_COMMAND_SIGN_VALUES[1],
        "wheel_effort_sign_left": WHEEL_EFFORT_SIGN_VALUES[0],
        "wheel_effort_sign_right": WHEEL_EFFORT_SIGN_VALUES[1],
        "lqr_state_left_position_m": ACTUATOR_COMMAND_DEBUG.lqr_state_position_m[0],
        "lqr_state_right_position_m": ACTUATOR_COMMAND_DEBUG.lqr_state_position_m[1],
        "lqr_state_left_velocity_m_s": ACTUATOR_COMMAND_DEBUG.lqr_state_velocity_m_s[0],
        "lqr_state_right_velocity_m_s": ACTUATOR_COMMAND_DEBUG.lqr_state_velocity_m_s[1],
        "lqr_state_pitch_rad": ACTUATOR_COMMAND_DEBUG.lqr_state_pitch_rad,
        "lqr_state_pitch_rate_rad_s": ACTUATOR_COMMAND_DEBUG.lqr_state_pitch_rate_rad_s,
        "force_left_position_n": ACTUATOR_COMMAND_DEBUG.lqr_force_position_n[0],
        "force_right_position_n": ACTUATOR_COMMAND_DEBUG.lqr_force_position_n[1],
        "force_left_velocity_n": ACTUATOR_COMMAND_DEBUG.lqr_force_velocity_n[0],
        "force_right_velocity_n": ACTUATOR_COMMAND_DEBUG.lqr_force_velocity_n[1],
        "force_left_pitch_n": ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_n[0],
        "force_right_pitch_n": ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_n[1],
        "force_left_pitch_rate_n": ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_rate_n[0],
        "force_right_pitch_rate_n": ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_rate_n[1],
        "force_left_sum_terms_n": (
            ACTUATOR_COMMAND_DEBUG.lqr_force_position_n[0]
            + ACTUATOR_COMMAND_DEBUG.lqr_force_velocity_n[0]
            + ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_n[0]
            + ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_rate_n[0]
        ),
        "force_right_sum_terms_n": (
            ACTUATOR_COMMAND_DEBUG.lqr_force_position_n[1]
            + ACTUATOR_COMMAND_DEBUG.lqr_force_velocity_n[1]
            + ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_n[1]
            + ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_rate_n[1]
        ),
        "force_total_position_n": (
            ACTUATOR_COMMAND_DEBUG.lqr_force_position_n[0] + ACTUATOR_COMMAND_DEBUG.lqr_force_position_n[1]
        ),
        "force_total_velocity_n": (
            ACTUATOR_COMMAND_DEBUG.lqr_force_velocity_n[0] + ACTUATOR_COMMAND_DEBUG.lqr_force_velocity_n[1]
        ),
        "force_total_pitch_n": (
            ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_n[0] + ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_n[1]
        ),
        "force_total_pitch_rate_n": (
            ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_rate_n[0] + ACTUATOR_COMMAND_DEBUG.lqr_force_pitch_rate_n[1]
        ),
        "force_left_lqr_physical_n": ACTUATOR_COMMAND_DEBUG.lqr_force_physical_n[0],
        "force_right_lqr_physical_n": ACTUATOR_COMMAND_DEBUG.lqr_force_physical_n[1],
        "force_total_phys_n": (
            ACTUATOR_COMMAND_DEBUG.lqr_force_physical_n[0] + ACTUATOR_COMMAND_DEBUG.lqr_force_physical_n[1]
        ),
        "current_left_lqr_physical_a": ACTUATOR_COMMAND_DEBUG.lqr_current_physical_a[0],
        "current_right_lqr_physical_a": ACTUATOR_COMMAND_DEBUG.lqr_current_physical_a[1],
        "current_left_lqr_raw_a": ACTUATOR_COMMAND_DEBUG.lqr_current_a[0],
        "current_right_lqr_raw_a": ACTUATOR_COMMAND_DEBUG.lqr_current_a[1],
        "lqr_current_equivalence_error_a": ACTUATOR_COMMAND_DEBUG.lqr_current_equivalence_error_a,
        "u_left_lqr_physical_a": ACTUATOR_COMMAND_DEBUG.lqr_current_physical_a[0],
        "u_right_lqr_physical_a": ACTUATOR_COMMAND_DEBUG.lqr_current_physical_a[1],
        "u_left_lqr": ACTUATOR_COMMAND_DEBUG.lqr_current_a[0],
        "u_right_lqr": ACTUATOR_COMMAND_DEBUG.lqr_current_a[1],
        "u_left_lqr_a": ACTUATOR_COMMAND_DEBUG.lqr_current_a[0],
        "u_right_lqr_a": ACTUATOR_COMMAND_DEBUG.lqr_current_a[1],
        "u_left_rl": ACTUATOR_COMMAND_DEBUG.rl_current_a[0],
        "u_right_rl": ACTUATOR_COMMAND_DEBUG.rl_current_a[1],
        "u_left_rl_a": ACTUATOR_COMMAND_DEBUG.rl_current_a[0],
        "u_right_rl_a": ACTUATOR_COMMAND_DEBUG.rl_current_a[1],
        "disturbance_current": args_cli.actuator_disturbance_current_a if args_cli.actuator_disturbance != "none" else 0.0,
        "disturbance_start": args_cli.actuator_disturbance_start_s,
        "actuator_disturbance_duration_s": args_cli.actuator_disturbance_duration_s if args_cli.actuator_disturbance == "step-forward" else 0.0,
        "disturbance_left_a": ACTUATOR_COMMAND_DEBUG.disturbance_current_a[0],
        "disturbance_right_a": ACTUATOR_COMMAND_DEBUG.disturbance_current_a[1],
        "current_left_after_lqr_before_disturbance_a": ACTUATOR_COMMAND_DEBUG.lqr_current_a[0],
        "current_right_after_lqr_before_disturbance_a": ACTUATOR_COMMAND_DEBUG.lqr_current_a[1],
        "current_left_final_command_a": ACTUATOR_COMMAND_DEBUG.commanded_current_a[0],
        "current_right_final_command_a": ACTUATOR_COMMAND_DEBUG.commanded_current_a[1],
        "u_left_final": ACTUATOR_COMMAND_DEBUG.commanded_current_a[0],
        "u_right_final": ACTUATOR_COMMAND_DEBUG.commanded_current_a[1],
        "u_left_final_a": ACTUATOR_COMMAND_DEBUG.commanded_current_a[0],
        "u_right_final_a": ACTUATOR_COMMAND_DEBUG.commanded_current_a[1],
        "u_left_cmd_a": ACTUATOR_COMMAND_DEBUG.commanded_current_a[0],
        "u_right_cmd_a": ACTUATOR_COMMAND_DEBUG.commanded_current_a[1],
        "reward": ACTUATOR_COMMAND_DEBUG.reward,
        "episode_reward": ACTUATOR_COMMAND_DEBUG.episode_reward,
        "base_position_x": state.root_position_world_m[0],
        "base_position_y": state.root_position_world_m[1],
        "base_position_z": state.root_position_world_m[2],
        "left_wheel_rpm": left_wheel_rpm,
        "right_wheel_rpm": right_wheel_rpm,
        "i_des_left": motor["i_des"][0], "i_des_right": motor["i_des"][1],
        "i_cmd_left": motor["i_cmd"][0], "i_cmd_right": motor["i_cmd"][1],
        "motor_current_desired_left_a": motor["i_des"][0],
        "motor_current_desired_right_a": motor["i_des"][1],
        "motor_current_saturated_left_a": motor["i_cmd"][0],
        "motor_current_saturated_right_a": motor["i_cmd"][1],
        "tau_actual_left": motor["tau_actual"][0], "tau_actual_right": motor["tau_actual"][1],
        "applied_effort_left_nm": motor["applied_effort"][0], "applied_effort_right_nm": motor["applied_effort"][1],
        "left_contact_force": contact_forces[0], "right_contact_force": contact_forces[1],
        "rpm_diff": rpm_diff,
        "current_diff": current_diff,
        "torque_diff": torque_diff,
        "contact_force_diff": contact_force_diff,
        "wheel_rpm_left": left_wheel_rpm,
        "wheel_rpm_right": right_wheel_rpm,
        "contact_force_left": contact_forces[0], "contact_force_right": contact_forces[1],
        "current_saturated_left": float(abs(motor["i_des"][0] - motor["i_cmd"][0]) > 1.0e-6),
        "current_saturated_right": float(abs(motor["i_des"][1] - motor["i_cmd"][1]) > 1.0e-6),
        "time_s": state.time_s,
        "left_wheel_position_rad": state.wheel_position_rad[0],
        "right_wheel_position_rad": state.wheel_position_rad[1],
        "left_wheel_position_physical_rad": state.wheel_position_rad[0],
        "right_wheel_position_physical_rad": state.wheel_position_rad[1],
        "raw_left_wheel_position_rad": state.raw_wheel_position_rad[0],
        "raw_right_wheel_position_rad": state.raw_wheel_position_rad[1],
        "left_wheel_velocity_rad_s": state.wheel_velocity_rad_s[0],
        "right_wheel_velocity_rad_s": state.wheel_velocity_rad_s[1],
        "left_wheel_velocity_physical_rad_s": state.wheel_velocity_rad_s[0],
        "right_wheel_velocity_physical_rad_s": state.wheel_velocity_rad_s[1],
        "raw_left_wheel_velocity_rad_s": state.raw_wheel_velocity_rad_s[0],
        "raw_right_wheel_velocity_rad_s": state.raw_wheel_velocity_rad_s[1],
        "left_i_des_a": motor["i_des"][0], "right_i_des_a": motor["i_des"][1],
        "left_i_cmd_a": motor["i_cmd"][0], "right_i_cmd_a": motor["i_cmd"][1],
        "left_wheel_velocity_raw_rad_s": motor["wheel_velocity_raw"][0], "right_wheel_velocity_raw_rad_s": motor["wheel_velocity_raw"][1],
        "left_wheel_velocity_used_rad_s": motor["wheel_velocity_used"][0], "right_wheel_velocity_used_rad_s": motor["wheel_velocity_used"][1],
        "left_wheel_velocity_used_rpm": motor["wheel_velocity_used"][0] * 60.0 / (2.0 * math.pi), "right_wheel_velocity_used_rpm": motor["wheel_velocity_used"][1] * 60.0 / (2.0 * math.pi),
        "left_omega_for_limiter_rad_s": motor["omega_for_limiter"][0], "right_omega_for_limiter_rad_s": motor["omega_for_limiter"][1],
        "left_tau_current_nm": motor["tau_current"][0], "right_tau_current_nm": motor["tau_current"][1],
        "left_tau_speed_limit_nm": motor["tau_speed_limit"][0], "right_tau_speed_limit_nm": motor["tau_speed_limit"][1],
        "left_tau_actual_nm": motor["tau_actual"][0], "right_tau_actual_nm": motor["tau_actual"][1],
        "left_applied_effort_nm": motor["applied_effort"][0], "right_applied_effort_nm": motor["applied_effort"][1],
        "root_x_m": state.root_position_world_m[0], "root_y_m": state.root_position_world_m[1], "root_z_m": state.root_position_world_m[2],
        "root_vx_m_s": state.root_linear_velocity_world_m_s[0], "root_vy_m_s": state.root_linear_velocity_world_m_s[1], "root_vz_m_s": state.root_linear_velocity_world_m_s[2],
        "gravity_body_x": state.projected_gravity_body[0], "gravity_body_y": state.projected_gravity_body[1], "gravity_body_z": state.projected_gravity_body[2],
        "angular_velocity_body_x_rad_s": state.angular_velocity_body_rad_s[0],
        "angular_velocity_body_y_rad_s": state.angular_velocity_body_rad_s[1],
        "angular_velocity_body_z_rad_s": state.angular_velocity_body_rad_s[2],
    }


PLOT_SIGNALS: dict[str, tuple[str, str]] = {
    "theta": ("Pitch angle", "rad"), "theta_dot": ("Pitch rate", "rad/s"),
    "yaw": ("Yaw error", "rad"), "yaw_rate": ("Yaw rate", "rad/s"),
    "base_yaw": ("Base yaw", "rad"), "base_yaw_rate": ("Base yaw rate", "rad/s"),
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
    "theta": ("theta", "theta_dot"),
    "yaw": ("yaw", "yaw_rate"),
    "base_yaw": ("base_yaw", "base_yaw_rate"),
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


# =============================================================================
# CSV LOGGING
# =============================================================================

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


# =============================================================================
# LIVE PLOTTING
# =============================================================================

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


# =============================================================================
# MAIN LOOP
# =============================================================================

@hydra_task_config(args_cli.task, None)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg: Any | None):
    if args_cli.list_signals:
        print_signal_names(); return
    if args_cli.sign_diagnostic and args_cli.test_mode != "free-spin":
        raise ValueError("--sign-diagnostic is only supported with --test-mode free-spin.")
    if args_cli.sign_diagnostic and args_cli.sign_diagnostic_current_a <= 0.0:
        raise ValueError("--sign-diagnostic-current-a must be positive.")
    if args_cli.sign_diagnostic and args_cli.sign_diagnostic_phase_time_s <= 0.0:
        raise ValueError("--sign-diagnostic-phase-time-s must be positive.")
    if args_cli.test_mode == "lqr-floor" and abs(args_cli.floor_initial_pitch_deg) > 35.0:
        raise ValueError("--floor-initial-pitch-deg magnitude must be at most 35 degrees.")
    if args_cli.test_mode == "lqr-floor" and args_cli.floor_stop_pitch_deg <= abs(args_cli.floor_initial_pitch_deg):
        old_stop = args_cli.floor_stop_pitch_deg
        args_cli.floor_stop_pitch_deg = abs(args_cli.floor_initial_pitch_deg) + args_cli.floor_auto_stop_margin_deg
        print(
            f"[SIM2SIM]: raised --floor-stop-pitch-deg from {old_stop:.3f} to "
            f"{args_cli.floor_stop_pitch_deg:.3f} so initial pitch does not stop immediately"
        )
    if args_cli.floor_auto_stop_margin_deg < 0.0:
        raise ValueError("--floor-auto-stop-margin-deg must be non-negative.")
    if args_cli.floor_settle_steps < 0:
        raise ValueError("--floor-settle-steps must be non-negative.")
    if args_cli.floor_contact_force_threshold < 0.0:
        raise ValueError("--floor-contact-force-threshold must be non-negative.")
    if args_cli.solver_position_iters < 1:
        raise ValueError("--solver-position-iters must be at least 1.")
    if args_cli.solver_velocity_iters < 0:
        raise ValueError("--solver-velocity-iters must be non-negative.")
    if args_cli.max_depenetration_velocity <= 0.0:
        raise ValueError("--max-depenetration-velocity must be positive.")
    if args_cli.mujoco_torque_limit <= 0.0:
        raise ValueError("--mujoco-torque-limit must be positive.")
    if args_cli.mujoco_wheel_damping < 0.0:
        raise ValueError("--mujoco-wheel-damping must be non-negative.")
    if args_cli.disturbance_start_s < 0.0:
        raise ValueError("--disturbance-start-s must be non-negative.")
    if args_cli.disturbance_duration_s < 0.0:
        raise ValueError("--disturbance-duration-s must be non-negative.")
    if args_cli.actuator_disturbance_start_s < 0.0:
        raise ValueError("--actuator-disturbance-start-s must be non-negative.")
    if args_cli.actuator_disturbance_samples < 0:
        raise ValueError("--actuator-disturbance-samples must be non-negative.")
    if args_cli.actuator_disturbance_duration_s < 0.0:
        raise ValueError("--actuator-disturbance-duration-s must be non-negative.")
    if args_cli.actuator_disturbance == "step-forward" and args_cli.actuator_disturbance_duration_s <= 0.0:
        raise ValueError("--actuator-disturbance-duration-s must be positive for --actuator-disturbance step-forward.")
    if args_cli.actuator_disturbance != "none" and args_cli.test_mode != "lqr-floor":
        raise ValueError("--actuator-disturbance is only supported with --test-mode lqr-floor.")
    if args_cli.residual_action_limit < 0.0:
        raise ValueError("--residual-action-limit must be non-negative.")
    if args_cli.residual_policy and not args_cli.enable_residual_rl:
        raise ValueError("--residual-policy requires --enable-residual-rl.")
    if args_cli.training_mode and args_cli.evaluation_mode:
        raise ValueError("--training-mode and --evaluation-mode are mutually exclusive.")
    if args_cli.test_mode in ("lqr-model", "lqr-floor"):
        print_lqr_diagnostic()
    if args_cli.enable_residual_rl:
        print(
            "[RESIDUAL RL]: ignored for scripts/lqr_control.py split 4-state LQR diagnostics. "
            "Residual current is forced to zero; LQR plus actuator disturbance remains active."
        )
    signals = [] if args_cli.no_plot else resolve_plot_signals(args_cli.plot_signals)
    env_cfg.scene.num_envs = 1
    env_cfg.decimation = args_cli.control_decimation
    env_cfg.sim.render_interval = args_cli.control_decimation
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else env_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device
    env_cfg.success_steps_required = 1_000_000_000; env_cfg.episode_length_s = 3600.0
    env_cfg.wheel_damping_scale_range = (1.0, 1.0)
    env_cfg.noise_proj_grav_std = 0.0; env_cfg.noise_ang_vel_std = 0.0; env_cfg.noise_cg_pos_std = 0.0
    env_cfg.robot_cfg.spawn.rigid_props.max_depenetration_velocity = args_cli.max_depenetration_velocity
    env_cfg.robot_cfg.spawn.articulation_props.solver_position_iteration_count = args_cli.solver_position_iters
    env_cfg.robot_cfg.spawn.articulation_props.solver_velocity_iteration_count = args_cli.solver_velocity_iters
    if "wheel_joints" in env_cfg.robot_cfg.actuators:
        env_cfg.robot_cfg.actuators["wheel_joints"].stiffness = 0.0
        env_cfg.robot_cfg.actuators["wheel_joints"].damping = args_cli.mujoco_wheel_damping
        if args_cli.actuator_model == "mujoco-torque":
            env_cfg.robot_cfg.actuators["wheel_joints"].effort_limit_sim = args_cli.mujoco_torque_limit
            env_cfg.robot_cfg.actuators["wheel_joints"].velocity_limit_sim = 1.0e6
    left_current = args_cli.motor_test_current if args_cli.left_motor_test_current is None else args_cli.left_motor_test_current
    right_current = args_cli.motor_test_current if args_cli.right_motor_test_current is None else args_cli.right_motor_test_current
    requested_current_limit = max(abs(left_current), abs(right_current))
    if args_cli.actuator_model == "mujoco-torque":
        requested_current_limit = args_cli.mujoco_torque_limit
    elif args_cli.test_mode == "lqr-model":
        requested_current_limit = abs(lqr_current_for_pitch_deg(args_cli.artificial_pitch_deg))
    elif args_cli.test_mode == "lqr-floor":
        requested_current_limit = 100.0
    # In MuJoCo-torque mode this field is reused as a torque normalizer [Nm].
    # In DDSM115 mode it remains the StandupEnv current normalizer [A].
    env_cfg.wheel_current_max = requested_current_limit if args_cli.actuator_model == "mujoco-torque" else max(DDSM115_I_PEAK, requested_current_limit)
    env_cfg.robot_cfg.init_state.pos = (0.0, 0.0, args_cli.base_height)
    if args_cli.test_mode == "lqr-floor":
        env_cfg.enable_wheel_contacts = True
        env_cfg.robot_cfg.spawn.activate_contact_sensors = True

    env = gym.make(args_cli.task, cfg=env_cfg)
    if args_cli.actuator_model == "mujoco-torque":
        install_mujoco_torque_actuator(env.unwrapped)
    resolve_disturbance_body(env.unwrapped)
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
    settled_steps, settled_contact_forces = settle_floor_contacts(env.unwrapped)
    global YAW_REFERENCE_RAD
    YAW_REFERENCE_RAD = yaw_from_quat_wxyz(read_state(env.unwrapped, 0.0).root_orientation_world_quat_wxyz)
    dt = env.unwrapped.step_dt
    start_sample = math.ceil(args_cli.actuator_disturbance_start_s / dt - 1.0e-9)
    ACTUATOR_COMMAND_DEBUG.start_sample_index = start_sample
    if args_cli.actuator_disturbance == "step-forward":
        duration_samples = max(1, math.ceil(args_cli.actuator_disturbance_duration_s / dt - 1.0e-9))
        ACTUATOR_COMMAND_DEBUG.stop_sample_index = start_sample + duration_samples
    else:
        ACTUATOR_COMMAND_DEBUG.stop_sample_index = start_sample + args_cli.actuator_disturbance_samples
    logger, plotter = CsvLogger(args_cli.log_csv), None
    if signals:
        try:
            plotter = LivePlot(signals, args_cli.plot_window)
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN]: Live plot disabled: {exc}")
    print(f"[INFO]: LQR diagnostic mode: {args_cli.test_mode}. Press Ctrl+C to stop.")
    print(f"[INFO]: dt={dt:.5f} s  rate={1.0 / dt:.1f} Hz")
    print(
        "[SIM2SIM]: "
        f"actuator_model={args_cli.actuator_model} lqr_output={args_cli.lqr_output} "
        f"wheel_normalizer={env.unwrapped.cfg.wheel_current_max:.6g} "
        f"solver_iters={args_cli.solver_position_iters}/{args_cli.solver_velocity_iters} "
        f"max_depenetration_velocity={args_cli.max_depenetration_velocity:.6g}"
    )
    if args_cli.actuator_model == "mujoco-torque":
        print(
            "[SIM2SIM]: MuJoCo matching active: "
            f"direct_torque_limit={args_cli.mujoco_torque_limit:.6g} Nm "
            f"wheel_damping={args_cli.mujoco_wheel_damping:.6g} Nm*s/rad "
            "DDSM115 current clamp and torque-speed limiter bypassed"
        )
    if args_cli.test_mode == "free-spin":
        print(f"[INFO]: fixed base height={args_cli.base_height:.3f} m")
        if args_cli.sign_diagnostic:
            total_time = 3.0 * args_cli.sign_diagnostic_phase_time_s
            print(
                "[SIGN DIAGNOSTIC]: +current left, coast, +current right; "
                f"current={args_cli.sign_diagnostic_current_a:+.3f} A phase_time={args_cli.sign_diagnostic_phase_time_s:.3f} s "
                f"auto_stop={total_time:.3f} s"
            )
        else:
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
        print(
            f"[CHECK]: floor contact settle steps={settled_steps} "
            f"left={settled_contact_forces[0]:.3f} N right={settled_contact_forces[1]:.3f} N "
            f"touching_floor={settled_contact_forces[0] > args_cli.floor_contact_force_threshold and settled_contact_forces[1] > args_cli.floor_contact_force_threshold}"
        )
    if args_cli.disturbance != "none":
        print(
            "[DISTURBANCE]: configured "
            f"mode={args_cli.disturbance} body={DISTURBANCE_STATE.body_names} ids={DISTURBANCE_STATE.body_ids.detach().cpu().tolist() if DISTURBANCE_STATE.body_ids is not None else []} "
            f"start={args_cli.disturbance_start_s:.3f} s duration={args_cli.disturbance_duration_s:.3f} s "
            f"force={args_cli.disturbance_force_n:.3f} N yaw_torque={args_cli.disturbance_yaw_torque_nm:.3f} Nm"
        )
    if args_cli.actuator_disturbance != "none":
        print(
            "[ACTUATOR DISTURBANCE]: configured "
            f"mode={args_cli.actuator_disturbance} "
            f"requested_start={args_cli.actuator_disturbance_start_s:.6f} s "
            f"actual_start_sample={ACTUATOR_COMMAND_DEBUG.start_sample_index} "
            f"actual_start_time={ACTUATOR_COMMAND_DEBUG.start_sample_index * dt:.6f} s "
            f"samples={max(0, ACTUATOR_COMMAND_DEBUG.stop_sample_index - ACTUATOR_COMMAND_DEBUG.start_sample_index)} "
            f"duration={max(0, ACTUATOR_COMMAND_DEBUG.stop_sample_index - ACTUATOR_COMMAND_DEBUG.start_sample_index) * dt:.6f} s "
            f"physical_forward={args_cli.actuator_disturbance_current_a:+.6f} A "
            f"mapping left={args_cli.actuator_disturbance_current_a:+.6f} A right={args_cli.actuator_disturbance_current_a:+.6f} A"
        )

    timestep, wall_start = 0, time.time()
    sign_diagnostic_last_phase = -1
    try:
        while simulation_app.is_running():
            start = time.time()
            with torch.inference_mode():
                state = read_state(env.unwrapped, timestep * dt)
                action = compute_action(state, timestep)
                if ACTUATOR_COMMAND_DEBUG.active and ACTUATOR_COMMAND_DEBUG.active_sample_index == 0:
                    print(
                        "[ACTUATOR DISTURBANCE]: active "
                        f"sample={ACTUATOR_COMMAND_DEBUG.sample_index} time={ACTUATOR_COMMAND_DEBUG.command_time_s:.6f} s "
                        f"u_lqr=[{ACTUATOR_COMMAND_DEBUG.lqr_current_a[0]:+.6f}, {ACTUATOR_COMMAND_DEBUG.lqr_current_a[1]:+.6f}] A "
                        f"u_rl=[{ACTUATOR_COMMAND_DEBUG.rl_current_a[0]:+.6f}, {ACTUATOR_COMMAND_DEBUG.rl_current_a[1]:+.6f}] A "
                        f"disturbance=[{ACTUATOR_COMMAND_DEBUG.disturbance_current_a[0]:+.6f}, {ACTUATOR_COMMAND_DEBUG.disturbance_current_a[1]:+.6f}] A "
                        f"u_cmd=[{ACTUATOR_COMMAND_DEBUG.commanded_current_a[0]:+.6f}, {ACTUATOR_COMMAND_DEBUG.commanded_current_a[1]:+.6f}] A"
                    )
                env_action = action_to_env(env.unwrapped, action)
                apply_disturbance(env.unwrapped, state.time_s)
                env.step(env_action)
                state = read_state(env.unwrapped, (timestep + 1) * dt)
                motor = read_motor_debug(env.unwrapped)
                contact_forces = read_wheel_contact_forces(env.unwrapped) if args_cli.test_mode == "lqr-floor" else (0.0, 0.0)
            timestep += 1
            row = sample_to_row(state, motor, contact_forces); logger.write(row)
            if args_cli.sign_diagnostic:
                phase_time = args_cli.sign_diagnostic_phase_time_s
                phase = min(int(row["time"] // phase_time), 3)
                if phase != sign_diagnostic_last_phase:
                    phase_name = ("left +current", "coast", "right +current", "done")[phase]
                    print(
                        "[SIGN DIAGNOSTIC] "
                        f"phase={phase_name} time={row['time']:.6f} s "
                        f"raw_vel=[{row['raw_left_wheel_velocity_rad_s']:+.6f}, {row['raw_right_wheel_velocity_rad_s']:+.6f}] rad/s "
                        f"phys_vel=[{row['left_wheel_velocity_physical_rad_s']:+.6f}, {row['right_wheel_velocity_physical_rad_s']:+.6f}] rad/s "
                        f"i_cmd=[{row['left_i_cmd_a']:+.6f}, {row['right_i_cmd_a']:+.6f}] A "
                        f"root_vx={row['root_vx_m_s']:+.6f} m/s yaw_rate={row['yaw_rate']:+.6f} rad/s"
                    )
                    sign_diagnostic_last_phase = phase
            tau_limit_diff = abs(motor["tau_speed_limit"][0] - motor["tau_speed_limit"][1])
            if tau_limit_diff > 0.2:
                print(
                    "[DDSM115 LIMITER ASYMMETRY] "
                    f"time={row['time']:.6f} s  "
                    f"left_omega_for_limiter={motor['omega_for_limiter'][0]:.6f} rad/s  "
                    f"right_omega_for_limiter={motor['omega_for_limiter'][1]:.6f} rad/s  "
                    f"left_tau_limit={motor['tau_speed_limit'][0]:.6f} Nm  "
                    f"right_tau_limit={motor['tau_speed_limit'][1]:.6f} Nm"
                )
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
            if args_cli.sign_diagnostic:
                stop_time = 3.0 * args_cli.sign_diagnostic_phase_time_s
                if row["time"] >= stop_time:
                    print(f"[STOP]: reached sign diagnostic time {stop_time:.3f} s.")
                    break
            if args_cli.test_mode == "free-spin" and args_cli.free_spin_test_time_s > 0.0 and row["time"] >= args_cli.free_spin_test_time_s:
                print(f"[STOP]: reached free-spin test time {args_cli.free_spin_test_time_s:.3f} s.")
                break
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
