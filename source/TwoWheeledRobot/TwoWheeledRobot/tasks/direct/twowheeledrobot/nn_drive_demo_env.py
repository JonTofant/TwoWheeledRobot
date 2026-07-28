"""Presentation environment for the NN drive policy — a visible training ground.

Inherits NNDriveEnv unchanged (same observation/action contract, motor model,
CyberGear stance processing and command anti-windup), and replaces only:

  * the scene: a ground plane plus one static "training pad" per environment,
    each with a run-up deck, a drop ledge, a hazard lip and a target zone,
  * the command source: the recording script writes per-robot joystick targets
    into ``demo_v_target`` / ``demo_w_target`` instead of the random sampler,
  * the environment origins: robots spawn on their deck, not on the flat grid.

Kept deliberately separate from ``nn_drive_env.py`` so a showcase scene can
never perturb the training task. ``_setup_scene`` is a near-copy of the parent's
rather than a super() call because the pads must be spawned *before*
``clone_environments``/``filter_collisions`` and be registered as global
collision geometry — otherwise the robots fall straight through them.
"""

from __future__ import annotations

from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sensors import Imu
from isaaclab.terrains import TerrainImporterCfg

from .nn_drive_demo_env_cfg import NNDriveDemoEnvCfg
from .nn_drive_env import NNDriveEnv
from .sim_params import GROUND_DYNAMIC_FRICTION, GROUND_STATIC_FRICTION

TRAINING_GROUND_PATH = "/World/training_ground"


class NNDriveDemoEnv(NNDriveEnv):
    cfg: NNDriveDemoEnvCfg

    def __init__(self, cfg: NNDriveDemoEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        # Joystick targets, written per robot by the recording script. The
        # command generator still slews them and integrates the position/heading
        # references exactly as the firmware does.
        self.demo_v_target = torch.zeros(self.num_envs, device=self.device)
        self.demo_w_target = torch.zeros(self.num_envs, device=self.device)
        print(
            f"[NNDriveDemoEnv] training ground: {self.num_envs} pads, "
            f"ledge={self.cfg.ledge_height_m * 100.0:.0f} cm, "
            f"deck top z={self.cfg.deck_top_z:.3f} m"
        )

    # ── Scene ────────────────────────────────────────────────────────────────

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot
        self.bno080 = Imu(self.cfg.bno080)
        self.scene.sensors["bno080"] = self.bno080

        self._ground_friction_randomization_mode = "inactive"
        self._ground_static_friction = GROUND_STATIC_FRICTION
        self._ground_dynamic_friction = GROUND_DYNAMIC_FRICTION

        terrain_cfg = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="plane",
            collision_group=-1,
            physics_material=self.cfg.terrain.physics_material,
            debug_vis=False,
        )
        terrain_cfg.physics_material.static_friction = GROUND_STATIC_FRICTION
        terrain_cfg.physics_material.dynamic_friction = GROUND_DYNAMIC_FRICTION
        terrain_cfg.num_envs = self.scene.cfg.num_envs
        terrain_cfg.env_spacing = self.scene.cfg.env_spacing
        self._terrain = terrain_cfg.class_type(terrain_cfg)

        self._pad_origins = self._compute_pad_origins()
        self._build_training_ground()
        self._place_env_origins()

        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[terrain_cfg.prim_path, TRAINING_GROUND_PATH])

        self._build_lighting()
        print(f"[NNDriveDemoEnv] pads spawned under {TRAINING_GROUND_PATH}, plane ground, custom 3-light rig")

    def _compute_pad_origins(self) -> torch.Tensor:
        """World XY of each pad reference point (lip centre), env 0 at the origin."""
        cols = max(1, int(self.cfg.pad_cols_x))
        origins = torch.zeros(self.num_envs, 2, device=self.device)
        for i in range(self.num_envs):
            origins[i, 0] = (i % cols) * self.cfg.pad_pitch_x_m
            origins[i, 1] = (i // cols) * self.cfg.pad_pitch_y_m
        return origins

    def _place_env_origins(self) -> None:
        """Spawn each robot on its own deck, ``run_up_x_m`` behind the lip."""
        origins = torch.zeros(self.num_envs, 3, device=self.device)
        origins[:, 0] = self._pad_origins[:, 0] - self.cfg.run_up_x_m
        origins[:, 1] = self._pad_origins[:, 1]
        origins[:, 2] = self.cfg.deck_top_z
        self._terrain.env_origins[:] = origins.to(self._terrain.env_origins.device)

    def _build_arena(self) -> None:
        """Dark slab under the whole pad grid — the floor the robots stand on."""
        cfg = self.cfg
        margin = cfg.arena_margin_m
        x_lo = float(self._pad_origins[:, 0].min()) - cfg.deck_length_x_m - margin
        x_hi = float(self._pad_origins[:, 0].max()) + cfg.landing_length_x_m + margin
        y_lo = float(self._pad_origins[:, 1].min()) - 0.5 * cfg.pad_width_y_m - margin
        y_hi = float(self._pad_origins[:, 1].max()) + 0.5 * cfg.pad_width_y_m + margin
        self._spawn_box(
            f"{TRAINING_GROUND_PATH}/arena",
            size=(x_hi - x_lo, y_hi - y_lo, cfg.arena_thickness_m),
            pos=(0.5 * (x_lo + x_hi), 0.5 * (y_lo + y_hi), 0.5 * cfg.arena_thickness_m),
            color=cfg.color_arena,
            roughness=cfg.floor_roughness,
            collide=True,
        )

    def _build_training_ground(self) -> None:
        cfg = self.cfg
        self._build_arena()
        # Everything on a pad is measured from the arena surface, not z = 0.
        z0 = cfg.arena_thickness_m
        plate_t = cfg.plate_thickness_m
        deck_h = cfg.deck_top_z - z0
        half_w = 0.5 * cfg.pad_width_y_m
        trim_t = 0.012           # trim bar cross-section
        tread_t = 0.004          # thin lighter cap on the deck for a tread look
        pad_x_lo = -cfg.deck_length_x_m
        pad_x_hi = cfg.landing_length_x_m
        pad_len = pad_x_hi - pad_x_lo
        pad_mid_x = 0.5 * (pad_x_lo + pad_x_hi)

        for i in range(self.num_envs):
            px = float(self._pad_origins[i, 0])
            py = float(self._pad_origins[i, 1])
            root = f"{TRAINING_GROUND_PATH}/pad_{i:02d}"

            # Landing-floor plate: the surface the robot lands on.
            self._spawn_box(
                f"{root}/floor",
                size=(pad_len, cfg.pad_width_y_m, plate_t),
                pos=(px + pad_mid_x, py, z0 + 0.5 * plate_t),
                color=cfg.color_plate,
                roughness=cfg.floor_roughness,
                collide=True,
            )
            # Run-up deck: solid block from the pad's back edge to the lip.
            self._spawn_box(
                f"{root}/deck",
                size=(cfg.deck_length_x_m, cfg.pad_width_y_m, deck_h),
                pos=(px - 0.5 * cfg.deck_length_x_m, py, z0 + 0.5 * deck_h),
                color=cfg.color_deck,
                roughness=cfg.floor_roughness,
                collide=True,
            )
            # Lighter tread cap so the deck surface reads separately from its
            # side walls in a still frame. Sits flush with the deck top.
            self._spawn_box(
                f"{root}/deck_tread",
                size=(cfg.deck_length_x_m - 0.02, cfg.pad_width_y_m - 0.02, tread_t),
                pos=(px - 0.5 * cfg.deck_length_x_m, py, z0 + deck_h - 0.5 * tread_t),
                color=cfg.color_deck_top,
                roughness=cfg.floor_roughness - 0.12,
                collide=False,
            )
            # Hazard stripe along the lip — the thing the robot drives over.
            self._spawn_box(
                f"{root}/lip_hazard",
                size=(0.045, cfg.pad_width_y_m - 0.02, 0.003),
                pos=(px - 0.024, py, z0 + deck_h + 0.0015),
                color=cfg.color_hazard,
                roughness=0.35,
                emissive=tuple(c * 0.30 for c in cfg.color_hazard),
                collide=False,
            )
            # Vertical hazard face on the ledge wall, so the drop height is
            # legible head-on.
            self._spawn_box(
                f"{root}/lip_face",
                size=(0.004, cfg.pad_width_y_m - 0.02, cfg.ledge_height_m * 0.55),
                pos=(px + 0.002, py, z0 + plate_t + cfg.ledge_height_m * 0.5),
                color=cfg.color_hazard,
                roughness=0.4,
                emissive=tuple(c * 0.18 for c in cfg.color_hazard),
                collide=False,
            )
            # Target zone on the landing floor: where the policy must come to
            # rest and hold station after the drop.
            self._spawn_box(
                f"{root}/target",
                size=(cfg.target_zone_size_m[0], cfg.target_zone_size_m[1], 0.002),
                pos=(px + cfg.target_zone_x_m, py, z0 + plate_t + 0.001),
                color=cfg.color_target,
                roughness=0.3,
                emissive=tuple(c * cfg.trim_emissive_scale for c in cfg.color_target),
                collide=False,
            )
            # Teal boundary trim: reads as one RL environment instance per tile.
            for name, size, pos in (
                ("trim_lo_y", (pad_len, trim_t, trim_t), (px + pad_mid_x, py - half_w, z0 + 0.5 * trim_t)),
                ("trim_hi_y", (pad_len, trim_t, trim_t), (px + pad_mid_x, py + half_w, z0 + 0.5 * trim_t)),
                ("trim_back", (trim_t, cfg.pad_width_y_m, trim_t), (px + pad_x_lo, py, z0 + 0.5 * trim_t)),
                ("trim_front", (trim_t, cfg.pad_width_y_m, trim_t), (px + pad_x_hi, py, z0 + 0.5 * trim_t)),
            ):
                self._spawn_box(
                    f"{root}/{name}",
                    size=size,
                    pos=pos,
                    color=cfg.color_trim,
                    roughness=0.3,
                    emissive=tuple(c * cfg.trim_emissive_scale for c in cfg.color_trim),
                    collide=False,
                )
            # Two obstacle blocks parked off the drive line: training props that
            # dress the pad without interfering with the landing.
            for j, (bx, by, bh) in enumerate(
                ((1.30, -0.45, 0.022), (1.30, 0.45, 0.016)),
            ):
                self._spawn_box(
                    f"{root}/bump_{j}",
                    size=(0.16, 0.34, bh),
                    pos=(px + bx, py + by, z0 + plate_t + 0.5 * bh),
                    color=cfg.color_bump,
                    roughness=cfg.floor_roughness - 0.15,
                    collide=True,
                )

    def _spawn_box(
        self,
        prim_path: str,
        size: tuple,
        pos: tuple,
        color: tuple,
        roughness: float = 0.5,
        metallic: float = 0.0,
        emissive: tuple = (0.0, 0.0, 0.0),
        collide: bool = False,
    ) -> None:
        """Spawn one static box. ``collide=False`` gives a visual-only decal."""
        box = sim_utils.CuboidCfg(
            size=size,
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=color,
                emissive_color=emissive,
                roughness=roughness,
                metallic=metallic,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg() if collide else None,
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=GROUND_STATIC_FRICTION,
                dynamic_friction=GROUND_DYNAMIC_FRICTION,
                restitution=0.0,
            )
            if collide
            else None,
        )
        box.func(prim_path, box, translation=pos)

    def _build_lighting(self) -> None:
        """Cool ambient + warm key + cool fill: directional shadows, no flat look."""
        cfg = self.cfg
        dome = sim_utils.DomeLightCfg(intensity=cfg.dome_intensity, color=cfg.dome_color)
        dome.func("/World/Light/dome", dome)
        sun = sim_utils.DistantLightCfg(
            intensity=cfg.sun_intensity,
            color=cfg.sun_color,
            angle=cfg.sun_angle_deg,
        )
        sun.func("/World/Light/key", sun, orientation=cfg.sun_orientation)
        fill = sim_utils.DistantLightCfg(
            intensity=cfg.fill_intensity,
            color=cfg.fill_color,
            angle=6.0,
        )
        fill.func("/World/Light/fill", fill, orientation=cfg.fill_orientation)

    # ── Commands come from the recording script, not the random sampler ───────

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if hasattr(self, "demo_v_target"):
            self._commands.v_target[:] = self.demo_v_target
            self._commands.w_target[:] = self.demo_w_target
        super()._pre_physics_step(actions)

    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        if hasattr(self, "demo_v_target"):
            self.demo_v_target.zero_()
            self.demo_w_target.zero_()
        # Keep the random sampler from ever firing during the clip.
        if hasattr(self, "_commands"):
            self._commands.v_target.zero_()
            self._commands.w_target.zero_()
            self._commands.next_resample_step[:] = 2**30
