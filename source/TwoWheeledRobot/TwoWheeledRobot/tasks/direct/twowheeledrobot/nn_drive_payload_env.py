"""Fixed-stance drive task with an object put on (and taken off) the platform.

Experiment only. The object is a rigid box fused to the platform body at a
random spot on the deck, at a random time, through the PhysX mass/COM/inertia
tensors -- see NNDrivePayloadEnvCfg for why this rather than a point force.
Ground truth (payload_mass_now, payload_pos_b) and the hardware-readable leg
sensor channels (leg_features) are exposed for the estimator scripts; the
policy observation is untouched, so the frozen balance GRU sees exactly the
13-value contract it was trained on.
"""

from __future__ import annotations

from collections.abc import Sequence

import torch

from pxr import Usd, UsdGeom

from .nn_drive_env import NNDriveEnv
from .nn_drive_payload_env_cfg import NNDrivePayloadEnvCfg


def _skew_parallel_axis(d: torch.Tensor) -> torch.Tensor:
    """|d|^2 I - d d^T for a batch of offsets (n, 3) -> (n, 3, 3)."""
    eye = torch.eye(3, dtype=d.dtype).expand(d.shape[0], 3, 3)
    return (d * d).sum(dim=1).view(-1, 1, 1) * eye - d.unsqueeze(2) * d.unsqueeze(1)


class NNDrivePayloadEnv(NNDriveEnv):
    cfg: NNDrivePayloadEnvCfg

    def __init__(self, cfg: NNDrivePayloadEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self._measure_deck()

        # Platform properties after the per-episode body randomization, before
        # any payload; the payload is composed on top of these (CPU, PhysX API).
        self._plat_mass0 = torch.zeros(n)
        self._plat_com0 = torch.zeros(n, 3)
        self._plat_inertia0 = torch.zeros(n, 3, 3)

        self.payload_mass = torch.zeros(n, device=dev)  # what will be / was placed
        self.payload_pos_b = torch.zeros(n, 3, device=dev)  # object COM, platform frame
        self.payload_size = torch.zeros(n, 3, device=dev)
        self.payload_on = torch.zeros(n, dtype=torch.bool, device=dev)
        self._place_step = torch.full((n,), -1, dtype=torch.long, device=dev)
        self._remove_step = torch.full((n,), -1, dtype=torch.long, device=dev)
        self._leg_torque_bias = torch.zeros(n, 4, device=dev)
        print(
            f"[NNDrivePayloadEnv] payload mass {cfg.payload_mass_kg_range} kg (absent p={cfg.payload_absent_prob}), "
            f"deck centre {self._deck_center_b.tolist()} m, half extent {self._deck_half_xy.tolist()} m, "
            f"top z {self._deck_top_z_b:.4f} m (platform frame)"
        )

    # ── Ground truth / features for the estimator ────────────────────────────

    @property
    def payload_mass_now(self) -> torch.Tensor:
        return torch.where(self.payload_on, self.payload_mass, torch.zeros_like(self.payload_mass))

    def leg_features(self) -> torch.Tensor:
        """(n, 8): measured CyberGear angles (rad) and torque estimates (Nm), fl fr bl br."""
        pos = self.robot.data.joint_pos[:, self._cg_ids]
        pos = pos + torch.randn_like(pos) * self.cfg.leg_pos_noise_std_rad
        pos = torch.round(pos / (2.0 * torch.pi / 16384.0)) * (2.0 * torch.pi / 16384.0)
        tau = self.robot.data.applied_torque[:, self._cg_ids]
        tau = tau + self._leg_torque_bias + torch.randn_like(tau) * self.cfg.leg_torque_noise_std_nm
        return torch.cat([pos, tau], dim=1)

    # ── Deck geometry ────────────────────────────────────────────────────────

    def _measure_deck(self) -> None:
        """Deck outline and top height in the platform body frame, from the USD bound.

        Done once at construction, when the USD pose and the PhysX pose are both
        the authored spawn pose. The platform body quaternion is identity there
        (verified 2026-08-05), so the world-frame bound differs from the body
        frame only by the body position.
        """
        root = self.sim.stage.GetPrimAtPath("/World/envs/env_0/Robot")
        prim = next((p for p in Usd.PrimRange(root) if p.GetName() == "Platform_Group"), None)
        if prim is None:
            raise RuntimeError("Platform_Group prim not found under /World/envs/env_0/Robot")
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        lo = torch.tensor(list(box.GetMin()), dtype=torch.float32)
        hi = torch.tensor(list(box.GetMax()), dtype=torch.float32)
        xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        origin = torch.tensor(list(xform.ExtractTranslation()), dtype=torch.float32)
        body_w = self.robot.data.body_pos_w[0, self._platform_body_col].detach().cpu()
        if (origin - body_w).abs().max() > 1e-3:
            raise RuntimeError(f"USD platform origin {origin.tolist()} != PhysX body pos {body_w.tolist()}")
        self._deck_center_b = (0.5 * (lo + hi) - origin)[:2]
        self._deck_half_xy = 0.5 * (hi - lo)[:2]
        self._deck_top_z_b = float(hi[2] - origin[2])

    # ── Sampling and applying the payload ────────────────────────────────────

    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        if not hasattr(self, "_place_step"):
            return
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.tensor(env_ids, device=self.device)
        n, dev, cfg = len(ids), self.device, self.cfg
        ids_cpu = ids.detach().cpu()
        col = self._platform_body_col

        view = self.robot.root_physx_view
        self._plat_mass0[ids_cpu] = view.get_masses()[ids_cpu, col]
        self._plat_com0[ids_cpu] = view.get_coms()[ids_cpu, col, :3]
        self._plat_inertia0[ids_cpu] = view.get_inertias()[ids_cpu, col].view(-1, 3, 3)

        mass = torch.empty(n, device=dev).uniform_(*cfg.payload_mass_kg_range)
        mass[torch.rand(n, device=dev) < cfg.payload_absent_prob] = 0.0
        size = torch.stack([
            torch.empty(n, device=dev).uniform_(*cfg.payload_size_x_m_range),
            torch.empty(n, device=dev).uniform_(*cfg.payload_size_y_m_range),
            torch.empty(n, device=dev).uniform_(*cfg.payload_size_z_m_range),
        ], dim=1)
        lim = (self._deck_half_xy.to(dev) - cfg.payload_edge_margin_m).view(1, 2)
        off = torch.stack([
            torch.empty(n, device=dev).uniform_(*cfg.payload_offset_x_range_m),
            torch.empty(n, device=dev).uniform_(*cfg.payload_offset_y_range_m),
        ], dim=1)
        off = torch.maximum(torch.minimum(off, lim), -lim)
        pos = torch.zeros(n, 3, device=dev)
        pos[:, :2] = self._deck_center_b.to(dev) + off
        pos[:, 2] = self._deck_top_z_b + 0.5 * size[:, 2]

        dt = self.step_dt
        place = (torch.empty(n, device=dev).uniform_(*cfg.payload_place_s_range) / dt).long()
        remove = place + (torch.empty(n, device=dev).uniform_(*cfg.payload_remove_after_s_range) / dt).long()
        remove[torch.rand(n, device=dev) >= cfg.payload_remove_prob] = -1

        self.payload_mass[ids] = mass
        self.payload_size[ids] = size
        self.payload_pos_b[ids] = pos
        self.payload_on[ids] = False
        self._place_step[ids] = torch.where(mass > 0, place, torch.full_like(place, -1))
        self._remove_step[ids] = torch.where(mass > 0, remove, torch.full_like(remove, -1))
        self._leg_torque_bias[ids] = torch.empty(n, 4, device=dev).uniform_(*cfg.leg_torque_bias_nm_range)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        step = self.episode_length_buf
        put_on = (step == self._place_step) & ~self.payload_on
        take_off = (step == self._remove_step) & self.payload_on
        if put_on.any():
            self.payload_on[put_on] = True
            self._write_platform(put_on.nonzero().flatten())
        if take_off.any():
            self.payload_on[take_off] = False
            self._write_platform(take_off.nonzero().flatten())
        super()._pre_physics_step(actions)

    def set_payload(self, env_ids: torch.Tensor, mass: torch.Tensor, offset_xy: torch.Tensor,
                    size: torch.Tensor, place_step: torch.Tensor, remove_step: torch.Tensor | None = None) -> None:
        """Override the sampled payload for evaluation grids (call right after reset)."""
        self.payload_mass[env_ids] = mass
        self.payload_size[env_ids] = size
        self.payload_pos_b[env_ids, :2] = self._deck_center_b.to(self.device) + offset_xy
        self.payload_pos_b[env_ids, 2] = self._deck_top_z_b + 0.5 * size[:, 2]
        self._place_step[env_ids] = torch.where(mass > 0, place_step, torch.full_like(place_step, -1))
        rem = remove_step if remove_step is not None else torch.full_like(place_step, -1)
        self._remove_step[env_ids] = torch.where(mass > 0, rem, torch.full_like(rem, -1))

    def _write_platform(self, ids: torch.Tensor) -> None:
        """Compose platform + payload (or restore the bare platform) for these envs."""
        ids_cpu = ids.detach().cpu()
        col = self._platform_body_col
        on = self.payload_on[ids].cpu()
        mp = torch.where(on, self.payload_mass[ids].cpu(), torch.zeros(len(ids)))
        p = self.payload_pos_b[ids].cpu()
        a, b, c = self.payload_size[ids].cpu().unbind(dim=1)
        m0, c0, i0 = self._plat_mass0[ids_cpu], self._plat_com0[ids_cpu], self._plat_inertia0[ids_cpu]

        m = m0 + mp
        com = (m0.unsqueeze(1) * c0 + mp.unsqueeze(1) * p) / m.unsqueeze(1)
        box = torch.zeros(len(ids), 3, 3)
        box[:, 0, 0] = mp / 12.0 * (b * b + c * c)
        box[:, 1, 1] = mp / 12.0 * (a * a + c * c)
        box[:, 2, 2] = mp / 12.0 * (a * a + b * b)
        inertia = (i0 + m0.view(-1, 1, 1) * _skew_parallel_axis(c0 - com)
                   + box + mp.view(-1, 1, 1) * _skew_parallel_axis(p - com))

        view = self.robot.root_physx_view
        masses = view.get_masses()
        masses[ids_cpu, col] = m
        view.set_masses(masses, ids_cpu)
        coms = view.get_coms()
        coms[ids_cpu, col, :3] = com
        view.set_coms(coms, ids_cpu)
        inertias = view.get_inertias()
        inertias[ids_cpu, col] = inertia.reshape(-1, 9)
        view.set_inertias(inertias, ids_cpu)
