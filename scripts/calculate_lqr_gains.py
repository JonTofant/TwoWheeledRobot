#!/usr/bin/env python3
"""Calculate editable LQR gains for scripts/lqr_control.py.

This script uses a small linear inverted-pendulum-on-wheels model for the
current LQR tuning setup where the CyberGear joints are held fixed.

State used for the pitch model:
    x = [wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s]

Input used by the model:
    u_force = forward ground force from both wheels, in newtons

The main printed gains are per-wheel current gains for the editable
manual_current block in scripts/lqr_control.py:
    current_a = -K_current @ state

The script also prints equivalent torque/current/action gains so you can check
the conversion chain.

Pure roll is not controllable by wheel torque in this simple fixed-leg model.
Differential wheel current can yaw/turn the robot, but it does not create a
clean linear roll restoring input, so K_ROLL and K_ROLL_RATE are reported as 0.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are


@dataclass(frozen=True)
class RobotParams:
    # Defaults match scripts/lqr_control.py. Wheel radius uses the measured
    # 100.7 mm wheel diameter for physical tuning.
    body_mass_kg: float = 2.60
    wheel_cart_mass_kg: float = 1.53
    wheel_radius_m: float = 0.05035
    body_com_height_m: float = 0.14
    body_pitch_inertia_kg_m2: float | None = 0.0129596588636
    body_height_m: float = 0.20
    body_length_m: float = 0.15
    no_load_current_a: float = 0.25
    no_load_speed_rpm: float = 200.0
    wheel_torque_constant_nm_per_a: float = 0.75
    wheel_current_max_a: float = 0.96 / 0.75
    gravity_m_s2: float = 9.81

    @property
    def total_mass_kg(self) -> float:
        return self.body_mass_kg + self.wheel_cart_mass_kg

    @property
    def pitch_inertia_kg_m2(self) -> float:
        if self.body_pitch_inertia_kg_m2 is not None:
            return self.body_pitch_inertia_kg_m2
        # Rough box inertia about the body CoM for pitch. Do not use m*l^2
        # here; the cart-pole equations already include the parallel-axis term.
        return self.body_mass_kg * (self.body_height_m**2 + self.body_length_m**2) / 12.0

    @property
    def wheel_internal_damping_nm_s_rad(self) -> float:
        omega_no_load_rad_s = self.no_load_speed_rpm * 2.0 * np.pi / 60.0
        return self.wheel_torque_constant_nm_per_a * self.no_load_current_a / omega_no_load_rad_s


@dataclass(frozen=True)
class LqrWeights:
    # Increase a Q value to care more about that state.
    # Increase R to make the controller less aggressive.
    q_position: float = 0.0
    q_wheel_velocity: float = 0.15
    q_pitch: float = 45.0
    q_pitch_rate: float = 2.0
    r_force: float = 1.0


def build_pitch_model(params: RobotParams) -> tuple[np.ndarray, np.ndarray]:
    """Return continuous-time A, B for a cart-pole-style wheel balance model."""
    m = params.body_mass_kg
    m_cart = params.wheel_cart_mass_kg
    l = params.body_com_height_m
    inertia = params.pitch_inertia_kg_m2
    g = params.gravity_m_s2

    # Convert per-wheel rotational damping into equivalent horizontal viscous
    # damping. Both wheels contribute: F_drag = -b * x_dot.
    b = 2.0 * params.wheel_internal_damping_nm_s_rad / (params.wheel_radius_m**2)

    denominator = inertia * (m_cart + m) + m_cart * m * l**2

    a22 = -((inertia + m * l**2) * b) / denominator
    a23 = (m**2 * g * l**2) / denominator
    a42 = -(m * l * b) / denominator
    a43 = (m * g * l * (m_cart + m)) / denominator

    b2 = (inertia + m * l**2) / denominator
    b4 = (m * l) / denominator

    a = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, a22, a23, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, a42, a43, 0.0],
        ],
        dtype=float,
    )
    b_mat = np.array([[0.0], [b2], [0.0], [b4]], dtype=float)
    return a, b_mat


def lqr(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Continuous-time LQR gain K for u = -Kx."""
    p = solve_continuous_are(a, b, q, r)
    return np.linalg.solve(r, b.T @ p)


def force_gain_to_normalized_action_gain(k_force: np.ndarray, params: RobotParams) -> np.ndarray:
    """Convert force gains to normalized equal-current wheel action gains."""
    # Equal wheel currents create forward force:
    #   F = (tau_left + tau_right) / radius = 2 * Kt * current / radius
    #   current = F * radius / (2 * Kt)
    #   normalized_action = current / current_max
    return k_force * params.wheel_radius_m / (
        2.0 * params.wheel_torque_constant_nm_per_a * params.wheel_current_max_a
    )


def print_results(params: RobotParams, weights: LqrWeights) -> None:
    a, b = build_pitch_model(params)
    q = np.diag(
        [
            weights.q_position,
            weights.q_wheel_velocity,
            weights.q_pitch,
            weights.q_pitch_rate,
        ]
    )
    r = np.array([[weights.r_force]], dtype=float)

    k_force = lqr(a, b, q, r).reshape(-1)
    k_action = force_gain_to_normalized_action_gain(k_force, params)

    print("Robot parameters used:")
    print(f"  body_mass_kg = {params.body_mass_kg}")
    print(f"  wheel_cart_mass_kg = {params.wheel_cart_mass_kg}")
    print(f"  total_mass_kg = {params.total_mass_kg}")
    print(f"  wheel_radius_m = {params.wheel_radius_m}")
    print(f"  body_com_height_m = {params.body_com_height_m}")
    print(f"  pitch_inertia_kg_m2 = {params.pitch_inertia_kg_m2}")
    print(f"  wheel_internal_damping_nm_s_rad = {params.wheel_internal_damping_nm_s_rad}")
    print()
    print("Continuous model state order:")
    print("  [wheel_position_m, wheel_velocity_m_s, pitch_rad, pitch_rate_rad_s]")
    print()
    print("LQR gain for force input, u_force_N = -K_force @ state:")
    print(f"  K_force = {k_force}")
    print()
    print("Converted gain for normalized wheel action, u_action = -K_action @ state:")
    print(f"  K_action = {k_action}")
    print()
    k_torque_per_wheel = k_force * params.wheel_radius_m * 0.5
    k_current_per_wheel = k_torque_per_wheel / params.wheel_torque_constant_nm_per_a

    print("Equivalent per-wheel torque gain, torque_nm = -K_torque @ state:")
    print(f"  K_torque_per_wheel = {k_torque_per_wheel}")
    print()
    print("Equivalent per-wheel current gain, current_a = -K_current @ state:")
    print(f"  K_current_per_wheel = {k_current_per_wheel}")
    print()
    print("Paste these into scripts/lqr_control.py manual_current USER LQR TUNING SECTION:")
    print(f"  K_WHEEL_POSITION_CURRENT = {k_current_per_wheel[0]:.6g}")
    print(f"  K_WHEEL_VELOCITY_CURRENT = {k_current_per_wheel[1]:.6g}")
    print(f"  K_PITCH_CURRENT = {k_current_per_wheel[2]:.6g}")
    print(f"  K_PITCH_RATE_CURRENT = {k_current_per_wheel[3]:.6g}")
    print("  K_ROLL_CURRENT = 0.0")
    print("  K_ROLL_RATE_CURRENT = 0.0")
    print()
    print("For reference, force gains for u_forward_force_N = -K_force @ state are:")
    print(f"  K_WHEEL_POSITION_FORCE = {k_force[0]:.6g}")
    print(f"  K_WHEEL_VELOCITY_FORCE = {k_force[1]:.6g}")
    print(f"  K_PITCH_FORCE = {k_force[2]:.6g}")
    print(f"  K_PITCH_RATE_FORCE = {k_force[3]:.6g}")
    print()
    print("For comparison, equivalent normalized Isaac action gains are:")
    print(f"  K_PITCH_ACTION = {k_action[2]:.6g}")
    print(f"  K_PITCH_RATE_ACTION = {k_action[3]:.6g}")
    print(f"  K_WHEEL_VELOCITY_ACTION_RAD_S = {k_action[1] * params.wheel_radius_m:.6g}")
    print()
    print("Notes:")
    print("  - Force gains assume pitch is rad and pitch_rate is rad/s.")
    print("  - lqr_control.py converts wheel joint rad/s to m/s before applying")
    print("    these current gains.")
    print("  - If the wheels drive the wrong way, flip signs in lqr_control.py first;")
    print("    do not immediately change the LQR weights.")


if __name__ == "__main__":
    print_results(RobotParams(), LqrWeights())
