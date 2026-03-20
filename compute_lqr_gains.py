"""
Offline LQR gain computation for a 2-Wheeled Inverted Pendulum (WIP).

Run this script ONCE (on your laptop / workstation with scipy):
    python compute_lqr_gains.py

Copy the printed K vector into TwowheeledrobotEnvCfg.lqr_K.

Theory
------
The 2D sagittal-plane model linearised around upright (θ=0, θ̇=0, v=0):

    State  x = [θ,  θ̇,  v,  pos]
    Input  u = symmetric wheel torque (Nm), same to both wheels

The coupled Euler-Lagrange equations give:

    [J_eq      -M_b*L ] [θ̈]   [M_b*g*L * θ]   [-1       ]
    [-M_b*L    M_eq   ] [v̇]  = [    0       ] + [2/R_wheel] * u

where
    J_eq  = I_b  + M_b*L²              body pitch inertia (about wheel axis)
    M_eq  = M_b  + 2*M_w + 2*I_w/R²   effective translational mass
    det   = J_eq * M_eq - (M_b*L)²

Continuous LQR: minimise ∫ (x'Qx + u'Ru) dt
    P = solution of Riccati equation
    K = R⁻¹ B' P
"""

import numpy as np
from scipy.linalg import solve_continuous_are

# ============================================================== #
# ▼▼▼  SET THESE TO YOUR ROBOT'S PHYSICAL PARAMETERS  ▼▼▼        #
# ============================================================== #

# Body (everything above the wheel axis)
M_b  = 5.0       # kg   — body mass
L    = 0.25      # m    — height of body CoM above wheel axis
I_b  = 0.12      # kg·m² — body pitch moment of inertia about its own CoM
# Note: J_eq = I_b + M_b*L² (parallel axis theorem, wheel axis as pivot)

# Wheels (one side; duplicated internally)
M_w  = 0.35      # kg   — mass of one wheel
R    = 0.08      # m    — wheel rolling radius
I_w  = 0.5 * M_w * R**2   # kg·m² — thin-disk approximation (solid disk = 0.5mR²)
# For a hollow-rim wheel use I_w = M_w * R² instead.

g = 9.81         # m/s²

# ============================================================== #
# ▼▼▼  LQR WEIGHT MATRICES — tune these  ▼▼▼                    #
#                                                                  #
#  Q penalises state deviation:                                    #
#    Q[0,0]  — pitch angle penalty      (large  → strongly upright)#
#    Q[1,1]  — pitch rate penalty       (medium → damp oscillation)#
#    Q[2,2]  — forward velocity penalty (small  → allow drift)     #
#    Q[3,3]  — position penalty         (small  → hold position)   #
#  R penalises control effort (small → aggressive, large → gentle) #
# ============================================================== #
Q = np.diag([200.0, 20.0, 5.0, 5.0])
R_mat = np.array([[0.5]])

# ============================================================== #
# Derived quantities
# ============================================================== #
J_eq  = I_b  + M_b * L**2
M_eq  = M_b  + 2*M_w + 2*I_w / R**2
det   = J_eq * M_eq - (M_b * L)**2

print("=" * 60)
print("Robot parameters")
print(f"  Body:  M_b={M_b} kg,  L={L} m,  I_b={I_b} kg·m²")
print(f"  Wheel: M_w={M_w} kg,  R={R} m,  I_w={I_w:.5f} kg·m²")
print(f"  J_eq = {J_eq:.4f} kg·m²")
print(f"  M_eq = {M_eq:.4f} kg")
print(f"  det  = {det:.4f}")
print("=" * 60)

# ============================================================== #
# State-space matrices (continuous time)                          #
# State:  x = [θ, θ̇, v, pos]                                    #
# Input:  u = wheel torque (Nm, applied to BOTH wheels)           #
# ============================================================== #
# From the coupled Lagrangian (after matrix inversion):
#   θ̈  = (M_eq * M_b*g*L * θ  +  M_b*L * u*(2/R)) / det
#   v̇   = (M_b*L * M_b*g*L * θ  -  J_eq * u*(2/R)) / det
# Position: ṗos = v
# (No coupling through θ in the linearised velocity row because
#  the gravity term in the translation equation vanishes at θ=0)

A = np.array([
    [0,   1,                      0,  0],
    [M_eq * M_b * g * L / det,   0,  0,  0],
    [M_b**2 * g * L**2 / (L * det),  0,  0,  0],   # = -(−M_b*L*M_b*g*L/det) CHECK
    [0,   0,                      1,  0],
])

# Correct derivation:
#   θ̈  row:  M_eq*M_b*g*L/det  * θ  −  (M_b*L * 2/R)/det  * u
#   v̇   row:  M_b*L*M_b*g*L/det * θ  −  (J_eq  * 2/R)/det  * u
#   Wait — sign of v̇ gravity: -(-M_b*L) in top row → +M_b*L * (M_b*g*L/det)
# Let's be explicit:
A = np.array([
    [0,  1,  0,  0],
    [M_eq * M_b * g * L / det,   0,  0,  0],
    [M_b * L * M_b * g * L / det,  0,  0,  0],
    [0,  0,  1,  0],
])

B = np.array([
    [0],
    [-(M_b * L * 2/R) / det],   # pitch acceleration per unit torque (note: -ve)
    [(J_eq  * 2/R) / det],       # forward acceleration per unit torque
    [0],
])

# The sign of B[1] (pitch channel): a forward torque (wheels push backward)
# produces a reaction that tilts the body backward → negative pitch acceleration.
# Flip signs if your physical convention differs.

print("A matrix:")
print(np.array2string(A, precision=4, suppress_small=True))
print("\nB matrix:")
print(np.array2string(B, precision=4, suppress_small=True))

# ============================================================== #
# Solve the continuous Riccati equation                           #
# ============================================================== #
try:
    P = solve_continuous_are(A, B, Q, R_mat)
    K = np.linalg.inv(R_mat) @ B.T @ P   # shape (1, 4)
    K_vec = K[0].tolist()

    print("\n" + "=" * 60)
    print("LQR gain vector K = [k_θ, k_θ̇, k_v, k_pos]")
    print(f"  K = {[round(k, 4) for k in K_vec]}")
    print("=" * 60)
    print("\nPaste this into TwowheeledrobotEnvCfg:")
    print(f"    lqr_K: list = {[round(k, 4) for k in K_vec]}")
    print()

    # Verify stability: closed-loop eigenvalues should all have negative real parts.
    A_cl = A - B @ K
    eigs = np.linalg.eigvals(A_cl)
    print("Closed-loop eigenvalues (should all have Re < 0):")
    for i, e in enumerate(eigs):
        stable = "✓" if e.real < 0 else "✗ UNSTABLE"
        print(f"  λ{i} = {e.real:.3f} + {e.imag:.3f}j   {stable}")

except np.linalg.LinAlgError as err:
    print(f"\nERROR: Riccati solver failed — {err}")
    print("Check that (A, B) is stabilisable and (A, Q^0.5) is detectable.")

# ============================================================== #
# Quick PD fallback reference                                     #
# For simple PD without position feedback:                        #
#   u = -kp_pitch * θ  -  kd_pitch * θ̇                         #
# Ziegler-Nichols starting point for a WIP:                      #
#   kp ≈ M_b*g*L / R  (enough to fight gravity with unit tilt)   #
#   kd ≈ kp * sqrt(J_eq / (M_b*g*L))  (critical damping)         #
# ============================================================== #
kp_approx = M_b * g * L / R
T_nat     = 2 * np.pi * np.sqrt(J_eq / (M_b * g * L))   # natural period
kd_approx = kp_approx * T_nat / (2 * np.pi)
print("\nPD fallback starting point:")
print(f"  kp_pitch ≈ {kp_approx:.1f} Nm/rad")
print(f"  kd_pitch ≈ {kd_approx:.1f} Nm/(rad/s)")
print(f"  (natural period of pendulum = {T_nat:.3f} s)")
