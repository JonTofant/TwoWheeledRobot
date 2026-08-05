#!/usr/bin/env python3
"""Export an inference-ready pure balance / drive controller ONNX model.

Inputs are the TorchScript and ONNX actor exported by scripts/rsl_rl/play.py.
This script appends the deployment contract final layer directly in ONNX:

  Balance policy (2 outputs, default):
      current_a = tanh(actor(obs)) * I_max

  Drive policy (--cg-outputs 4, 6 outputs total):
      commands  = tanh(actor(obs)) * [auth, auth, auth, auth, I_max, I_max]
      → outputs [0-3] are CyberGear position targets in rad (firmware must
        still clamp to joint limits and slew-limit at cg_target_slew_radps),
        outputs [4-5] are left/right DDSM115 currents in A.

Appending ONNX nodes avoids retracing Isaac Lab's TorchScript policy exporter,
which is not traceable as a child module in some Isaac/PyTorch builds.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import torch


def _scale_vector(cg_outputs: int, cg_authority_rad: float, i_max_a: float) -> list[float]:
    return [cg_authority_rad] * cg_outputs + [i_max_a] * 2


def append_scaled_output(actor_onnx_path: Path, output_path: Path, scale: list[float], output_name: str) -> None:
    import numpy as np
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    model = onnx.load(str(actor_onnx_path))
    graph = model.graph
    if len(graph.output) != 1:
        raise SystemExit(f"Expected one actor output in {actor_onnx_path}, found {len(graph.output)}")

    actor_output = graph.output[0]
    actor_output_name = actor_output.name
    tanh_output_name = actor_output_name + "_tanh"
    scale_name = "output_scale"

    graph.node.append(helper.make_node("Tanh", inputs=[actor_output_name], outputs=[tanh_output_name], name="out_tanh"))
    scale_arr = np.asarray(scale, dtype=np.float32)
    if len(scale_arr) == 2 and scale_arr[0] == scale_arr[1]:
        # Keep the legacy scalar initializer for 2-output balance policies so
        # existing STM32 parsers keep working.
        scale_arr = np.array(scale_arr[0], dtype=np.float32)
    graph.initializer.append(numpy_helper.from_array(scale_arr, name=scale_name))
    graph.node.append(
        helper.make_node("Mul", inputs=[tanh_output_name, scale_name], outputs=[output_name], name="out_scale")
    )

    output_type = actor_output.type.tensor_type
    new_output = helper.make_tensor_value_info(output_name, TensorProto.FLOAT, None)
    new_output.type.tensor_type.shape.CopyFrom(output_type.shape)
    graph.output.remove(actor_output)
    graph.output.append(new_output)
    onnx.checker.check_model(model)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output_path))


def validate(
    torchscript_path: Path, onnx_path: Path, obs_dim: int, samples: int, tolerance: float, scale: list[float]
) -> float:
    import numpy as np

    try:
        import onnxruntime as ort
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "onnxruntime is not installed; exported ONNX was written but numerical validation was skipped. "
            "Install onnxruntime to validate exports."
        ) from exc

    torch_policy = torch.jit.load(str(torchscript_path), map_location="cpu").eval()
    scale_t = torch.tensor(scale, dtype=torch.float32)
    obs = torch.randn(samples, obs_dim, dtype=torch.float32).clamp(-3.0, 3.0)
    with torch.inference_mode():
        torch_out = (torch.tanh(torch_policy(obs)) * scale_t).cpu().numpy()
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    # Isaac Lab's ONNX actor export uses dynamic_axes={}, so the graph's batch
    # dimension is fixed at 1 (from the traced dummy input) — run one sample
    # at a time rather than as a single batched call.
    obs_np = obs.cpu().numpy().astype(np.float32)
    onnx_out = np.concatenate([session.run(None, {input_name: obs_np[i : i + 1]})[0] for i in range(samples)], axis=0)
    max_error = float(np.max(np.abs(torch_out - onnx_out)))
    if max_error >= tolerance:
        raise SystemExit(f"ONNX validation failed: max_error={max_error:.8g} >= {tolerance:.8g}")
    return max_error


def main() -> None:
    parser = argparse.ArgumentParser(description="Export pure NN current/drive policy to ONNX and validate it.")
    parser.add_argument("--policy", required=True, type=Path, help="TorchScript actor exported as policy.pt")
    parser.add_argument(
        "--actor-onnx",
        type=Path,
        default=None,
        help="Raw actor ONNX exported by play.py. Defaults to policy.onnx next to --policy.",
    )
    parser.add_argument("--output", required=True, type=Path, help="Output ONNX path")
    parser.add_argument("--obs-dim", type=int, default=8, help="8 for PureNNBalance, 20 for NNDrive")
    parser.add_argument("--i-max-a", type=float, default=2.0)
    parser.add_argument(
        "--cg-outputs",
        type=int,
        default=0,
        help="Number of leading CyberGear outputs (0 for balance policies, 4 for NNDrive).",
    )
    parser.add_argument(
        "--cg-authority-rad",
        type=float,
        default=math.pi / 2,
        help="CyberGear tanh scale (rad). Must equal NNDriveEnvCfg.cg_action_authority_rad.",
    )
    parser.add_argument("--samples", type=int, default=256)
    parser.add_argument("--tolerance", type=float, default=1.0e-4)
    parser.add_argument(
        "--require-validation",
        action="store_true",
        help="Fail if ONNX Runtime validation cannot run. By default missing onnxruntime only warns after export.",
    )
    args = parser.parse_args()

    actor_onnx = args.actor_onnx if args.actor_onnx is not None else args.policy.with_suffix(".onnx")
    if not actor_onnx.is_file():
        raise SystemExit(f"Actor ONNX not found: {actor_onnx}. Run scripts/rsl_rl/play.py first or pass --actor-onnx.")

    scale = _scale_vector(args.cg_outputs, args.cg_authority_rad, args.i_max_a)
    output_name = "commands" if args.cg_outputs > 0 else "current_a"
    append_scaled_output(actor_onnx, args.output, scale, output_name)
    print(f"Exported {args.output} (outputs={len(scale)}, scale={scale})")
    try:
        max_error = validate(args.policy, args.output, args.obs_dim, args.samples, args.tolerance, scale)
    except RuntimeError as exc:
        if args.require_validation:
            raise SystemExit(str(exc)) from exc
        print(f"WARNING: {exc}")
        print("Install with: python -m pip install onnxruntime")
    else:
        print(f"ONNX validation passed: max_error={max_error:.8g} < {args.tolerance:.8g}")


if __name__ == "__main__":
    main()
