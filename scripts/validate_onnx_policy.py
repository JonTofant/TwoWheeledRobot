#!/usr/bin/env python3
"""Validate a TorchScript policy against an ONNX policy on identical observations."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare TorchScript and ONNX policy outputs.")
    parser.add_argument("--torchscript", required=True, type=Path)
    parser.add_argument("--onnx", required=True, type=Path)
    parser.add_argument("--obs-dim", type=int, default=8)
    parser.add_argument("--samples", type=int, default=256)
    parser.add_argument("--tolerance", type=float, default=1.0e-4)
    args = parser.parse_args()

    import onnxruntime as ort

    torch_policy = torch.jit.load(str(args.torchscript), map_location="cpu").eval()
    obs = torch.randn(args.samples, args.obs_dim, dtype=torch.float32).clamp(-3.0, 3.0)
    with torch.inference_mode():
        torch_out = torch_policy(obs).cpu().numpy()

    session = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    onnx_out = session.run(None, {input_name: obs.cpu().numpy().astype(np.float32)})[0]
    max_error = float(np.max(np.abs(torch_out - onnx_out)))
    print(f"max_error={max_error:.8g}")
    if max_error >= args.tolerance:
        raise SystemExit(f"validation failed: max_error >= {args.tolerance}")


if __name__ == "__main__":
    main()
