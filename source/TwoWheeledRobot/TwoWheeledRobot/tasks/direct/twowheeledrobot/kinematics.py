"""
Leg kinematics — ported from kinematics.c, unchanged.

Provides:
  set_leg_foot_position(xf, yf)        two-link IK for one leg pair
  cybergear_stance_angles(N, device)   equilibrium joint angles as Isaac tensor
"""

from __future__ import annotations

import math
import torch

# ── Geometry constants (cm — matches kinematics.c) ────────────────────────────
L1_C: float          = 10.8   # base link width between pivot points
L2_C: float          = 10.0   # upper leg link length
L3_C: float          = 20.0   # lower leg link length
BASE_TARGET_Y: float = 16.0   # default foot height below body


def _solve_ik_two_solutions(
    base_x: float, base_y: float,
    foot_x: float, foot_y: float,
    L_upper: float, L_lower: float,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    dx   = foot_x - base_x
    dy   = foot_y - base_y
    d_sq = dx * dx + dy * dy
    d    = math.sqrt(d_sq)
    if d > (L_upper + L_lower) or d < abs(L_upper - L_lower):
        return None
    a  = (L_upper * L_upper - L_lower * L_lower + d_sq) / (2.0 * d)
    h  = math.sqrt(max(L_upper * L_upper - a * a, 0.0))
    mx = base_x + a * dx / d
    my = base_y + a * dy / d
    rx = -dy * (h / d)
    ry =  dx * (h / d)
    return (mx + rx, my + ry), (mx - rx, my - ry)


def set_leg_foot_position(xf: float, yf: float) -> tuple[float, float] | None:
    """
    Two-link IK for one leg pair.

    Parameters
    ----------
    xf, yf : foot position in cm (same frame as kinematics.c)

    Returns
    -------
    (angle_front, angle_back) in rad, or None if unreachable.
    """
    sol_B = _solve_ik_two_solutions(L1_C, 0.0, xf, yf, L2_C, L3_C)
    sol_C = _solve_ik_two_solutions(0.0,  0.0, xf, yf, L2_C, L3_C)
    if sol_B is None or sol_C is None:
        return None
    (B_x1, B_y1), (B_x2, B_y2) = sol_B
    (C_x1, C_y1), (C_x2, C_y2) = sol_C
    cB_x, cB_y = (B_x1, B_y1) if B_x1 >= B_x2 else (B_x2, B_y2)
    cC_x, cC_y = (C_x1, C_y1) if C_x1 <= C_x2 else (C_x2, C_y2)
    alpha1 = math.atan2(cB_y,        cB_x - L1_C)
    alpha2 = math.atan2(cC_y - 0.0,  cC_x - 0.0)
    return -alpha1, -(alpha2 + math.pi)


def cybergear_stance_angles(num_envs: int, device: str) -> torch.Tensor:
    """
    IK equilibrium joint angles for all four CyberGear motors [fl, fr, bl, br].

    On the physical robot setMechanicalZero() is called at this pose, making
    motor angle = 0 the equilibrium.  In Isaac the USD zero is the mesh default
    (not the equilibrium), so this offset is added in the translation layer so
    that desired_angle = 0 means the same thing in both places.

    Returns
    -------
    Tensor of shape (num_envs, 4) in rad.
    """
    ik = set_leg_foot_position(L1_C * 0.5, BASE_TARGET_Y)
    if ik is not None:
        lf0, lb0 = ik
        eq = [-lf0, lf0, -lb0, lb0]   # left side is USD-mirrored → negate
    else:
        eq = [0.0, 0.0, 0.0, 0.0]
    return torch.tensor([eq], device=device, dtype=torch.float32).expand(num_envs, -1)
