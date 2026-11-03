"""Payload-mass estimator network (shared by training and closed-loop evaluation).

A small GRU over the per-step robot signals, run at the 66.7 Hz control rate
alongside the balance policy. Heads: payload mass (kg) and, as an auxiliary
target that forces the network to separate "how heavy" from "where", the
object's deck offset (m). Inputs are standardized inside the module so the
saved model is self-contained.
"""

from __future__ import annotations

import torch
from torch import nn

FEATURE_GROUPS = {
    # Only what the deployed firmware already computes for the balance policy.
    "imu_wheels": list(range(0, 15)),
    # Plus CyberGear feedback (angle + torque), which needs a CAN read-back.
    "imu_wheels_legs": list(range(0, 23)),
}


class PayloadMassEstimator(nn.Module):
    def __init__(self, feature_ids: list[int], hidden: int = 32, mean=None, std=None):
        super().__init__()
        f = len(feature_ids)
        self.register_buffer("feature_ids", torch.tensor(feature_ids, dtype=torch.long))
        self.register_buffer("mean", torch.zeros(f) if mean is None else mean.float())
        self.register_buffer("std", torch.ones(f) if std is None else std.float())
        self.rnn = nn.GRU(f, hidden)
        self.head = nn.Sequential(nn.Linear(hidden, 32), nn.ReLU(), nn.Linear(32, 3))
        self.hidden_size = hidden

    def forward(self, x: torch.Tensor, h: torch.Tensor | None = None):
        """x: (T, B, 23) full feature vector. Returns mass (T, B), offset (T, B, 2), h."""
        x = (x[..., self.feature_ids] - self.mean) / self.std
        out, h = self.rnn(x, h)
        y = self.head(out)
        return y[..., 0], y[..., 1:3], h

    @torch.no_grad()
    def step(self, x: torch.Tensor, h: torch.Tensor | None):
        """One control tick for B envs: x (B, 23) -> mass (B,), offset (B, 2), h."""
        mass, offset, h = self.forward(x.unsqueeze(0), h)
        return mass[0].clamp_min(0.0), offset[0], h

    def num_params(self) -> int:
        return sum(p.numel() for p in self.parameters())
