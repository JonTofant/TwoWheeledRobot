"""The hardware-deployed balance GRU, frozen, with a per-env hidden state.

The archived TorchScript export (exported/policy.pt) carries a (1, 1, 64)
hidden-state buffer, so it can only drive one env at a time. This rebuilds the
deterministic actor (GRU 13->64, then a [64, 64] ReLU head -> 2 wheel actions)
directly from the training checkpoint so thousands of envs can run in lockstep,
and resets hidden state per env exactly as the firmware does on (re)arm.
"""

from __future__ import annotations

from pathlib import Path

import torch
from torch import nn

REPO = Path(__file__).resolve().parents[2]

# Arm C, range + GRU, training seed 42 -- the policy flashed on the robot for the
# ERK/actuators paper (ExportedPolicy/fixedstance_range_gru_2026-08-10/MANIFEST.md;
# firmware ONNX is byte-identical to this run's exported/policy_drive.onnx).
HARDWARE_GRU_RUN = REPO / "ExportedPolicy/fixedstance_range_gru_2026-08-10/2026-08-10_23-51-29_range_gru_stage5"
HARDWARE_GRU_CHECKPOINT = HARDWARE_GRU_RUN / "model_775.pt"
HARDWARE_GRU_EXPORT = HARDWARE_GRU_RUN / "exported/policy.pt"


class FrozenGruPolicy(nn.Module):
    def __init__(self, checkpoint: Path, num_envs: int, device: str):
        super().__init__()
        state = torch.load(checkpoint, map_location=device, weights_only=False)["model_state_dict"]
        w_ih = state["memory_a.rnn.weight_ih_l0"]
        hidden, num_obs = w_ih.shape[0] // 3, w_ih.shape[1]
        self.rnn = nn.GRU(num_obs, hidden, num_layers=1)
        self.rnn.load_state_dict({k.removeprefix("memory_a.rnn."): v for k, v in state.items()
                                  if k.startswith("memory_a.rnn.")})
        layer_ids = sorted({int(k.split(".")[1]) for k in state if k.startswith("actor.")})
        layers: list[nn.Module] = []
        for i in layer_ids:
            w, b = state[f"actor.{i}.weight"], state[f"actor.{i}.bias"]
            lin = nn.Linear(w.shape[1], w.shape[0])
            lin.load_state_dict({"weight": w, "bias": b})
            layers += [lin, nn.ReLU()]
        self.actor = nn.Sequential(*layers[:-1])  # no activation after the output layer
        self.num_obs = num_obs
        self.to(device).eval()
        self.register_buffer("hidden", torch.zeros(1, num_envs, hidden, device=device))

    @torch.no_grad()
    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        out, self.hidden = self.rnn(obs.unsqueeze(0), self.hidden)
        return self.actor(out.squeeze(0))

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        if env_ids is None:
            self.hidden.zero_()
        else:
            self.hidden[:, env_ids] = 0.0
