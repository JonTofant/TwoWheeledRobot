"""Presentation scene for the NN drive policy: a grid of identical training pads.

This is a *showcase* configuration, not a training task. It reuses the NNDrive
observation/action contract and motor model unchanged (so what you see is the
same controller that trains and deploys), and only replaces the terrain with a
hand-built "training ground": one pad per environment, each with a raised deck,
a drop ledge, a hazard-striped lip and a target zone on the landing floor.

Geometry is in metres and the robot is small (10 cm wheels, 6.9 cm axle height),
so a 10 cm ledge is a two-wheel-radius drop.

Pad-local frame: the pad reference point is the **lip centre at floor level**.
    deck:    x in [-deck_length_x_m, 0], top at plate_thickness_m + ledge_height_m
    spawn:   x = -run_up_x_m, on the deck
    landing: x in [0, landing_length_x_m], top at plate_thickness_m
Positive robot velocity drives +x, i.e. from the deck out over the lip.
"""

import math

from isaaclab.utils import configclass

from .nn_drive_env_cfg import NNDriveEnvCfg


@configclass
class NNDriveDemoEnvCfg(NNDriveEnvCfg):
    # Long enough for establishing shot + drop + recovery + hold; the recording
    # script decides the actual clip length.
    episode_length_s: float = 60.0

    # The pads are static props spawned on top of a ground plane, so the
    # generated bumps/slopes terrain is not used here.
    terrain_mode: str = "flat"

    # Commands are written directly by the recording script; stage 5 only sets
    # the (unused) random-sampling limits and keeps the printout honest.
    curriculum_stage: int = 5
    benchmark_disturbance_kind: str = "none"  # no pushes/payloads in the hero shot
    enable_force_noise: bool = False

    # A hero shot should be reproducible and the robots should all start poised
    # facing their ledge, not at random headings.
    reset_yaw_random: bool = False
    reset_pitch_range_deg: float = 2.0
    reset_pitch_rate_range_radps: float = 0.0
    reset_velocity_range_mps: float = 0.0

    # The drop briefly pitches the robot well past the 25 deg training fall
    # threshold, which would terminate the episode mid-air and reset the robot
    # out of frame. Raise the bar so the clip shows the real recovery; the
    # recording script reports the measured pitch trace so "it survived" stays
    # a measurement, not an assumption.
    # Pinned back to the tilt rule: NNDriveEnvCfg switched to fall_mode="contact"
    # on 2026-08-07, which would end the hero shot the moment a leg brushed the
    # landing floor during the drop recovery. The showcase wants the clip to run.
    fall_mode: str = "tilt"
    fall_pitch_threshold_deg: float = 80.0
    fall_total_tilt_threshold_deg: float = 80.0

    # ── Pad geometry ─────────────────────────────────────────────────────────
    ledge_height_m: float = 0.10        # the drop, deck top above landing floor
    deck_length_x_m: float = 0.90       # run-up deck, lip at pad x = 0
    landing_length_x_m: float = 1.60    # floor beyond the lip
    pad_width_y_m: float = 1.30
    plate_thickness_m: float = 0.006    # landing-floor plate above the ground plane
    run_up_x_m: float = 0.20            # robot spawn, behind the lip
    target_zone_x_m: float = 0.75       # centre of the "hold here" marker after the drop
    target_zone_size_m: tuple = (0.34, 0.34)

    # ── Pad grid layout ──────────────────────────────────────────────────────
    # env 0 is the hero pad at the origin; +x is the drive direction, pads
    # recede along +y away from the camera.
    pad_cols_x: int = 2                 # pads side by side along the drive axis
    pad_pitch_x_m: float = 3.10
    pad_pitch_y_m: float = 1.95

    # ── Arena ────────────────────────────────────────────────────────────────
    # A dark slab under the whole pad grid. Without it the scene sits on Isaac's
    # infinite light-grey ground plane, which washes out the pads and puts a
    # horizon line through every frame. The slab is the collision surface the
    # robots actually use; the plane survives underneath, 2 cm lower, and only
    # shows outside the arena where it reads as intentional negative space.
    arena_thickness_m: float = 0.022
    # Generous overhang: the ground plane's grid texture kept reappearing at the
    # horizon behind the pads at 0.95 m and again at 5 m, so the slab is sized to
    # run past the far edge of every framing instead.
    arena_margin_m: float = 12.00        # slab overhang beyond the outermost pad

    # ── Look ─────────────────────────────────────────────────────────────────
    # Surfaces render considerably lighter than their diffuse values under the
    # key light, so the palette is pitched much darker than it reads on paper,
    # and floors are near-matte — at roughness ~0.6 they mirrored the dome and
    # the whole frame turned into blue-grey haze.
    color_arena: tuple = (0.012, 0.015, 0.022)      # near-black arena floor
    color_plate: tuple = (0.028, 0.034, 0.046)      # pad landing floor
    color_deck: tuple = (0.055, 0.062, 0.078)       # gunmetal run-up deck
    color_deck_top: tuple = (0.085, 0.095, 0.115)   # lighter tread surface
    color_hazard: tuple = (0.95, 0.42, 0.05)        # safety orange lip stripe
    color_trim: tuple = (0.04, 0.62, 0.70)          # teal env-boundary trim
    color_target: tuple = (0.10, 0.85, 0.65)        # green-cyan target zone
    color_bump: tuple = (0.20, 0.22, 0.26)          # obstacle blocks
    trim_emissive_scale: float = 0.55               # trim/target glow
    floor_roughness: float = 0.92                   # matte: no mirrored dome
    # Lighting: dark studio backdrop, one warm key for shadow direction, one
    # cool fill from behind for rim separation. The key was 2600 and blew the
    # deck surfaces out to near-white in render tests.
    dome_intensity: float = 260.0
    dome_color: tuple = (0.16, 0.20, 0.28)          # dark blue-grey backdrop
    sun_intensity: float = 2500.0
    sun_color: tuple = (1.0, 0.955, 0.90)           # warm key light
    sun_angle_deg: float = 1.0                      # crisper shadow edges
    sun_elevation_deg: float = 46.0
    sun_azimuth_deg: float = 38.0
    fill_intensity: float = 260.0
    fill_color: tuple = (0.42, 0.70, 1.0)           # cool rim/fill from the far side
    fill_elevation_deg: float = 26.0
    fill_azimuth_deg: float = 215.0

    def __post_init__(self):
        if hasattr(super(), "__post_init__"):
            super().__post_init__()
        self.viewer.resolution = (1920, 1080)
        # A deterministic hero shot: keep the motor model and sensor contract,
        # drop the per-episode randomization that would make robots differ.
        self.body_mass_scale_range = (1.0, 1.0)
        self.com_offset_y_range_m = (0.0, 0.0)
        self.com_offset_z_range_m = (0.0, 0.0)
        self.odometry_scale_range = (1.0, 1.0)
        self.pitch_bias_rad_range = (0.0, 0.0)
        self.pitch_rate_bias_radps_range = (0.0, 0.0)
        self.yaw_rate_bias_radps_range = (0.0, 0.0)
        self.payload_pitch_torque_nm_range = (0.0, 0.0)
        self.cg_calib_bias_rad_range = (0.0, 0.0)

    # ── Derived geometry ─────────────────────────────────────────────────────

    @property
    def deck_top_z(self) -> float:
        """Height of the run-up deck surface above the ground plane."""
        return self.arena_thickness_m + self.plate_thickness_m + self.ledge_height_m

    @property
    def sun_orientation(self) -> tuple:
        return _light_quat(self.sun_elevation_deg, self.sun_azimuth_deg)

    @property
    def fill_orientation(self) -> tuple:
        return _light_quat(self.fill_elevation_deg, self.fill_azimuth_deg)


def _light_quat(elevation_deg: float, azimuth_deg: float) -> tuple:
    """Quaternion (w, x, y, z) aiming a distant light from a sun position.

    A USD distant light emits along its local -Z. Pitching by
    ``-(90 - elevation)`` tips that direction up off straight-down, and the yaw
    rotation swings it to the requested compass bearing, so the light travels
    downward *and* horizontally and the scene gets directional shadows.
    """
    pitch = -math.radians(90.0 - elevation_deg)
    yaw = math.radians(azimuth_deg)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    # Rz(yaw) * Ry(pitch)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)
