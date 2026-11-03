"""Export a trained payload-mass estimator to a single-step ONNX graph for the MCU.

Inputs: features (1, F) -- the selected feature columns, raw units, in
FEATURE_GROUPS order -- and h_in (1, 1, H). Outputs: mass_kg (1, 1), offset_xy_m
(1, 2) and h_out (1, 1, H). Standardization is folded into the graph; the
firmware feeds h_out back as h_in next tick and zeros it on (re)arm.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parent))
from estimator import PayloadMassEstimator  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--estimator", type=Path, required=True)
parser.add_argument("--output", type=Path, default=None)
args = parser.parse_args()


class SingleStep(nn.Module):
    def __init__(self, model: PayloadMassEstimator):
        super().__init__()
        self.model = model

    def forward(self, features, h_in):
        m = self.model
        x = ((features - m.mean) / m.std).unsqueeze(0)
        out, h_out = m.rnn(x, h_in)
        y = m.head(out[0])
        return torch.relu(y[:, 0:1]), y[:, 1:3], h_out


def main():
    ckpt = torch.load(args.estimator, map_location="cpu", weights_only=False)
    model = PayloadMassEstimator(ckpt["feature_ids"], ckpt["hidden"])
    model.load_state_dict(ckpt["state_dict"])
    model.eval()
    wrapper = SingleStep(model).eval()
    f, h = len(ckpt["feature_ids"]), ckpt["hidden"]
    out = args.output or args.estimator.with_suffix(".onnx")
    torch.onnx.export(wrapper, (torch.zeros(1, f), torch.zeros(1, 1, h)), str(out),
                      input_names=["features", "h_in"], output_names=["mass_kg", "offset_xy_m", "h_out"],
                      opset_version=17, dynamo=False)
    # Numerical check of the single-step graph against the sequence model.
    import onnxruntime as ort

    sess = ort.InferenceSession(str(out))
    torch.manual_seed(0)
    seq = torch.randn(200, 1, 23) * 0.3
    ref, _, _ = model(seq)
    hh = torch.zeros(1, 1, h).numpy()
    worst = 0.0
    for t in range(seq.shape[0]):
        x = seq[t, :, model.feature_ids].numpy()
        mass, _, hh = sess.run(None, {"features": x, "h_in": hh})
        worst = max(worst, abs(float(mass[0, 0]) - max(float(ref[t, 0]), 0.0)))
    print(f"wrote {out}: {f} features, hidden {h}, {model.num_params()} params; max |onnx - torch| = {worst:.2e}")


if __name__ == "__main__":
    main()
