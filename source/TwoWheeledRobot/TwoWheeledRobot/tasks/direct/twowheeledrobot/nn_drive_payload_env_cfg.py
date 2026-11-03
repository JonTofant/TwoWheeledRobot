"""Payload-on-platform variant of the fixed-stance drive task.

Experiment only (branch experiment/payload-mass-estimator): an object of unknown
mass is put on the deck at an unknown spot while the frozen hardware GRU
balances, and a separate estimator network has to recover the object's mass.
Never trained against by PPO -- the balance policy is frozen.
"""

from __future__ import annotations

from isaaclab.utils import configclass

from .nn_drive_env_cfg import NNDriveFixedStanceEnvCfg


@configclass
class NNDrivePayloadEnvCfg(NNDriveFixedStanceEnvCfg):
    # ── Payload object ───────────────────────────────────────────────────────
    # The object is modelled as a rigid box fused to the platform body: its mass
    # is added at its own COM and its inertia (box + parallel axis) is added to
    # the platform's. A point force alone (m*g at the contact spot) reproduces
    # the gravity torque but omits the object's inertia, and that is not small
    # here: 1 kg sitting ~0.18 m above the axle adds ~0.03 kg m^2 of pitch
    # inertia about the axle, about as much as the whole 3.8 kg robot has.
    # A book that does not slide is rigidly attached for all practical purposes.
    payload_mass_kg_range: tuple = (0.1, 1.5)
    # Fraction of episodes with nothing ever placed (label stays 0 throughout).
    payload_absent_prob: float = 0.15
    # Object COM position on the deck, relative to the deck centre, platform
    # body frame: X lateral, Y fore/aft. Clipped per object so its COM stays at
    # least payload_edge_margin_m inside the deck outline (beyond that it tips off).
    payload_offset_x_range_m: tuple = (-0.10, 0.10)
    payload_offset_y_range_m: tuple = (-0.09, 0.09)
    payload_edge_margin_m: float = 0.012
    # Box dimensions for the inertia term (lateral, fore/aft, height). Book-ish.
    payload_size_x_m_range: tuple = (0.08, 0.22)
    payload_size_y_m_range: tuple = (0.06, 0.20)
    payload_size_z_m_range: tuple = (0.01, 0.08)
    # When the object is put on, and (with payload_remove_prob) taken off again.
    payload_place_s_range: tuple = (2.0, 6.0)
    payload_remove_prob: float = 0.25
    payload_remove_after_s_range: tuple = (5.0, 9.0)

    # ── Measured deck geometry (platform body frame) ─────────────────────────
    # Filled at env construction from the Platform_Group USD bound; see
    # NNDrivePayloadEnv._measure_deck. Not configuration.

    # ── Leg sensor model, for estimator features only ────────────────────────
    # CyberGear MIT-mode feedback the firmware can read back over CAN: joint
    # angle and the motor's torque estimate. ASSUMED magnitudes, no bench data:
    # 14-bit encoder (0.38 mrad steps) plus a little noise; torque from Iq with
    # ~0.03 Nm noise and a per-episode per-joint offset of up to +-0.05 Nm.
    leg_pos_noise_std_rad: float = 0.0005
    leg_torque_noise_std_nm: float = 0.03
    leg_torque_bias_nm_range: tuple = (-0.05, 0.05)

    # The balance policy's +-15% whole-robot mass randomization is training DR
    # for an unknown build. For estimating what is ON the robot the robot's own
    # mass must be known (it can be weighed): +-15% of 3.8 kg is +-0.57 kg of
    # irreducible label ambiguity. +-3% stands for weighing/model error. ASSUMED.
    body_mass_scale_range: tuple = (0.97, 1.03)

    # No pushes or invisible payload torques: the payload is the disturbance.
    benchmark_disturbance_kind: str = "none"
    curriculum_stage: int = 2
