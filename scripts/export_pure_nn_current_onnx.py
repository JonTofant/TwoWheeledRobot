#!/usr/bin/env python3
"""Export an inference-ready pure balance controller ONNX model.

Inputs are the TorchScript and ONNX actor exported by scripts/rsl_rl/play.py.
This script appends the deployment contract final layer directly in ONNX:

    current_a = tanh(actor(obs)) * I_max

Appending ONNX nodes avoids retracing Isaac Lab's TorchScript policy exporter,
which is not traceable as a child module in some Isaac/PyTorch builds.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


def append_current_output(actor_onnx_path: Path, output_path: Path, i_max_a: float) -> None:
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    import numpy as np

    model = onnx.load(str(actor_onnx_path))
    graph = model.graph
    if len(graph.output) != 1:
        raise SystemExit(f"Expected one actor output in {actor_onnx_path}, found {len(graph.output)}")

    actor_output = graph.output[0]
    actor_output_name = actor_output.name
    tanh_output_name = actor_output_name + "_tanh"
    scale_name = "i_max_a"
    current_output_name = "current_a"

    graph.node.append(helper.make_node("Tanh", inputs=[actor_output_name], outputs=[tanh_output_name], name="current_tanh"))
    graph.initializer.append(numpy_helper.from_array(np.array(i_max_a, dtype=np.float32), name=scale_name))
    graph.node.append(
        helper.make_node("Mul", inputs=[tanh_output_name, scale_name], outputs=[current_output_name], name="current_scale")
    )

    output_type = actor_output.type.tensor_type
    new_output = helper.make_tensor_value_info(current_output_name, TensorProto.FLOAT, None)
    new_output.type.tensor_type.shape.CopyFrom(output_type.shape)
    graph.output.remove(actor_output)
    graph.output.append(new_output)
    onnx.checker.check_model(model)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output_path))


def validate(torchscript_path: Path, onnx_path: Path, obs_dim: int, samples: int, tolerance: float, i_max_a: float) -> float:
    import numpy as np

    try:
        import onnxruntime as ort
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "onnxruntime is not installed; exported ONNX was written but numerical validation was skipped. "
            "Install onnxruntime to validate exports."
        ) from exc

    torch_policy = torch.jit.load(str(torchscript_path), map_location="cpu").eval()
    obs = torch.randn(samples, obs_dim, dtype=torch.float32).clamp(-3.0, 3.0)
    with torch.inference_mode():
        torch_out = (torch.tanh(torch_policy(obs)) * i_max_a).cpu().numpy()
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    onnx_out = session.run(None, {input_name: obs.cpu().numpy().astype(np.float32)})[0]
    max_error = float(np.max(np.abs(torch_out - onnx_out)))
    if max_error >= tolerance:
        raise SystemExit(f"ONNX validation failed: max_error={max_error:.8g} >= {tolerance:.8g}")
    return max_error


def main() -> None:
    parser = argparse.ArgumentParser(description="Export pure NN current policy to ONNX and validate it.")
    parser.add_argument("--policy", required=True, type=Path, help="TorchScript actor exported as policy.pt")
    parser.add_argument(
        "--actor-onnx",
        type=Path,
        default=None,
        help="Raw actor ONNX exported by play.py. Defaults to policy.onnx next to --policy.",
    )
    parser.add_argument("--output", required=True, type=Path, help="Output ONNX path")
    parser.add_argument("--obs-dim", type=int, default=8)
    parser.add_argument("--i-max-a", type=float, default=2.0)
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

    append_current_output(actor_onnx, args.output, args.i_max_a)
    print(f"Exported {args.output}")
    try:
        max_error = validate(args.policy, args.output, args.obs_dim, args.samples, args.tolerance, args.i_max_a)
    except RuntimeError as exc:
        if args.require_validation:
            raise SystemExit(str(exc)) from exc
        print(f"WARNING: {exc}")
        print("Install with: python -m pip install onnxruntime")
    else:
        print(f"ONNX validation passed: max_error={max_error:.8g} < {args.tolerance:.8g}")


if __name__ == "__main__":
    main()
