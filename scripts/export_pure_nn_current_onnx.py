#!/usr/bin/env python3
"""Export an inference-ready pure balance / drive controller ONNX model.

Inputs are the TorchScript and ONNX actor exported by scripts/rsl_rl/play.py.
This script appends the deployment contract final layer directly in ONNX:

  Balance policy (2 outputs, default):
      current_a = tanh(actor(obs)) * I_max

  Drive policy (--cg-outputs 4, 6 outputs total):
      t = tanh(actor(obs))
      cg_target[i] = t[i] * upper[i] if t[i] >= 0 else -t[i] * lower[i]
      wheel_current = t[4:6] * I_max
      → outputs [0-3] are CyberGear position targets in rad (firmware must
        still defensively clamp to the same limits and slew-limit at
        cg_target_slew_radps),
        outputs [4-5] are left/right DDSM115 currents in A.

Appending ONNX nodes avoids retracing Isaac Lab's TorchScript policy exporter,
which is not traceable as a child module in some Isaac/PyTorch builds.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import torch

DEFAULT_CG_LOWER_RAD = [math.radians(-10.0), math.radians(-90.0), math.radians(-90.0), math.radians(-10.0)]
DEFAULT_CG_UPPER_RAD = [math.radians(90.0), math.radians(10.0), math.radians(10.0), math.radians(90.0)]


class OnnxRuntimeUnavailable(RuntimeError):
    """Raised only when numerical validation cannot start without onnxruntime."""


def _deployment_output(
    actor_output: torch.Tensor,
    cg_outputs: int,
    i_max_a: float,
    cg_lower_rad: list[float],
    cg_upper_rad: list[float],
) -> torch.Tensor:
    squashed = torch.tanh(actor_output)
    if cg_outputs == 0:
        return squashed * i_max_a
    lower = torch.tensor(cg_lower_rad, dtype=squashed.dtype, device=squashed.device)
    upper = torch.tensor(cg_upper_rad, dtype=squashed.dtype, device=squashed.device)
    cg_action = squashed[:, :cg_outputs]
    cg_target = torch.where(cg_action >= 0.0, cg_action * upper, (-cg_action) * lower)
    wheel_current = squashed[:, cg_outputs:] * i_max_a
    return torch.cat([cg_target, wheel_current], dim=1)


def append_deployment_output(
    actor_onnx_path: Path,
    output_path: Path,
    cg_outputs: int,
    i_max_a: float,
    cg_lower_rad: list[float],
    cg_upper_rad: list[float],
    output_name: str,
) -> bool:
    """Returns True if the source graph was recurrent (h_out passed through unchanged)."""
    import numpy as np
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    model = onnx.load(str(actor_onnx_path))
    graph = model.graph
    # Isaac Lab's exporter (isaaclab_rl/rsl_rl/exporter.py::_OnnxPolicyExporter)
    # emits ["actions"] for a plain ActorCritic, ["actions", "h_out"] for GRU, or
    # ["actions", "h_out", "c_out"] for LSTM -- the action is always output[0],
    # any remaining outputs are recurrent state the caller must thread back in
    # as the next call's h_in/c_in input and are otherwise untouched here (this
    # deployment contract only transforms the action).
    if len(graph.output) not in (1, 2, 3):
        raise SystemExit(f"Expected 1-3 actor outputs in {actor_onnx_path}, found {len(graph.output)}")
    is_recurrent = len(graph.output) > 1

    actor_output = graph.output[0]
    actor_output_name = actor_output.name
    tanh_output_name = actor_output_name + "_tanh"

    graph.node.append(helper.make_node("Tanh", inputs=[actor_output_name], outputs=[tanh_output_name], name="out_tanh"))
    if cg_outputs == 0:
        # Keep the legacy scalar initializer for 2-output balance policies so
        # existing STM32 parsers keep working.
        scale_name = "output_scale"
        graph.initializer.append(numpy_helper.from_array(np.array(i_max_a, dtype=np.float32), name=scale_name))
        graph.node.append(
            helper.make_node("Mul", inputs=[tanh_output_name, scale_name], outputs=[output_name], name="out_scale")
        )
    else:
        cg_indices_name = "cg_output_indices"
        wheel_indices_name = "wheel_output_indices"
        lower_name = "cg_lower_rad"
        upper_name = "cg_upper_rad"
        wheel_scale_name = "wheel_output_scale"
        graph.initializer.extend(
            [
                numpy_helper.from_array(np.arange(cg_outputs, dtype=np.int64), name=cg_indices_name),
                numpy_helper.from_array(np.arange(cg_outputs, cg_outputs + 2, dtype=np.int64), name=wheel_indices_name),
                numpy_helper.from_array(np.asarray(cg_lower_rad, dtype=np.float32), name=lower_name),
                numpy_helper.from_array(np.asarray(cg_upper_rad, dtype=np.float32), name=upper_name),
                numpy_helper.from_array(np.array(i_max_a, dtype=np.float32), name=wheel_scale_name),
            ]
        )
        graph.node.extend(
            [
                helper.make_node(
                    "Gather", inputs=[tanh_output_name, cg_indices_name], outputs=["cg_tanh"], name="cg_select", axis=1
                ),
                helper.make_node(
                    "Gather",
                    inputs=[tanh_output_name, wheel_indices_name],
                    outputs=["wheel_tanh"],
                    name="wheel_select",
                    axis=1,
                ),
                helper.make_node("Relu", inputs=["cg_tanh"], outputs=["cg_positive"], name="cg_positive_part"),
                helper.make_node("Neg", inputs=["cg_tanh"], outputs=["cg_negated"], name="cg_negate"),
                helper.make_node("Relu", inputs=["cg_negated"], outputs=["cg_negative"], name="cg_negative_part"),
                helper.make_node(
                    "Mul", inputs=["cg_positive", upper_name], outputs=["cg_positive_rad"], name="cg_scale_positive"
                ),
                helper.make_node(
                    "Mul", inputs=["cg_negative", lower_name], outputs=["cg_negative_rad"], name="cg_scale_negative"
                ),
                helper.make_node(
                    "Add", inputs=["cg_positive_rad", "cg_negative_rad"], outputs=["cg_target_rad"], name="cg_target"
                ),
                helper.make_node(
                    "Mul", inputs=["wheel_tanh", wheel_scale_name], outputs=["wheel_current_a"], name="wheel_scale"
                ),
                helper.make_node(
                    "Concat",
                    inputs=["cg_target_rad", "wheel_current_a"],
                    outputs=[output_name],
                    name="commands_concat",
                    axis=1,
                ),
            ]
        )

    output_type = actor_output.type.tensor_type
    new_output = helper.make_tensor_value_info(output_name, TensorProto.FLOAT, None)
    new_output.type.tensor_type.shape.CopyFrom(output_type.shape)
    # Replace output[0] (the raw action) with the transformed one IN PLACE,
    # so recurrent state outputs (h_out, [c_out]) keep their original trailing
    # order instead of getting shuffled behind it by a remove()+append().
    recurrent_state_outputs = list(graph.output)[1:]
    del graph.output[:]
    graph.output.append(new_output)
    graph.output.extend(recurrent_state_outputs)
    onnx.checker.check_model(model)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output_path))
    return is_recurrent


def validate(
    torchscript_path: Path,
    onnx_path: Path,
    obs_dim: int,
    samples: int,
    tolerance: float,
    cg_outputs: int,
    i_max_a: float,
    cg_lower_rad: list[float],
    cg_upper_rad: list[float],
) -> float:
    import numpy as np

    try:
        import onnxruntime as ort
    except ModuleNotFoundError as exc:
        raise OnnxRuntimeUnavailable(
            "onnxruntime is not installed; exported ONNX was written but numerical validation was skipped. "
            "Install onnxruntime to validate exports."
        ) from exc

    torch_policy = torch.jit.load(str(torchscript_path), map_location="cpu").eval()
    obs = torch.randn(samples, obs_dim, dtype=torch.float32).clamp(-3.0, 3.0)
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    obs_np = obs.cpu().numpy().astype(np.float32)
    # Isaac Lab's ONNX actor export uses dynamic_axes={}, so the graph's batch
    # dimension is fixed at 1 (from the traced dummy input) — run one sample
    # at a time rather than as a single batched call.
    num_recurrent_inputs = len(session.get_inputs()) - 1
    if num_recurrent_inputs == 0:
        with torch.inference_mode():
            torch_out = (
                _deployment_output(torch_policy(obs), cg_outputs, i_max_a, cg_lower_rad, cg_upper_rad).cpu().numpy()
            )
        input_name = session.get_inputs()[0].name
        onnx_out = np.concatenate(
            [session.run(None, {input_name: obs_np[i : i + 1]})[0] for i in range(samples)], axis=0
        )
    else:
        # Recurrent: the TorchScript export keeps hidden state as an internal
        # buffer (single-argument forward(x), reset() zeros it), while the
        # ONNX export is functional (explicit h_in -> (actions, h_out), same
        # for c_in/c_out on LSTM) -- see isaaclab_rl/rsl_rl/exporter.py's
        # _TorchPolicyExporter vs _OnnxPolicyExporter. Comparable only if both
        # start from a zeroed state and see the same observation sequence in
        # the same order, so this steps through samples one at a time instead
        # of validating them independently.
        if hasattr(torch_policy, "reset"):
            torch_policy.reset()
        onnx_input_names = [inp.name for inp in session.get_inputs()]
        state = [np.zeros(inp.shape, dtype=np.float32) for inp in session.get_inputs()[1:]]
        torch_steps, onnx_steps = [], []
        with torch.inference_mode():
            for i in range(samples):
                torch_steps.append(
                    _deployment_output(
                        torch_policy(obs[i : i + 1]), cg_outputs, i_max_a, cg_lower_rad, cg_upper_rad
                    )
                    .cpu()
                    .numpy()
                )
                feeds = dict(zip(onnx_input_names, [obs_np[i : i + 1], *state]))
                outputs = session.run(None, feeds)
                onnx_steps.append(outputs[0])
                state = list(outputs[1:])
        torch_out = np.concatenate(torch_steps, axis=0)
        onnx_out = np.concatenate(onnx_steps, axis=0)
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
    parser.add_argument(
        "--obs-dim", type=int, default=8, help="8 for PureNNBalance, 21 for NNDrive, 13 for NNDriveFixedStance"
    )
    parser.add_argument("--i-max-a", type=float, default=2.0)
    parser.add_argument(
        "--cg-outputs",
        type=int,
        default=0,
        help="Number of leading CyberGear outputs (0 for balance policies, 4 for NNDrive).",
    )
    parser.add_argument(
        "--cg-lower-rad",
        type=float,
        nargs=4,
        default=DEFAULT_CG_LOWER_RAD,
        metavar=("FL", "FR", "BL", "BR"),
        help="CyberGear lower joint limits in radians (fl fr bl br).",
    )
    parser.add_argument(
        "--cg-upper-rad",
        type=float,
        nargs=4,
        default=DEFAULT_CG_UPPER_RAD,
        metavar=("FL", "FR", "BL", "BR"),
        help="CyberGear upper joint limits in radians (fl fr bl br).",
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
    if args.cg_outputs not in (0, 4):
        parser.error("--cg-outputs must be 0 for balance or 4 for NNDrive")

    output_name = "commands" if args.cg_outputs > 0 else "current_a"
    is_recurrent = append_deployment_output(
        actor_onnx,
        args.output,
        args.cg_outputs,
        args.i_max_a,
        args.cg_lower_rad,
        args.cg_upper_rad,
        output_name,
    )
    output_count = args.cg_outputs + 2
    recurrent_note = " + recurrent state (h_out[, c_out]) passed through unchanged" if is_recurrent else ""
    print(
        f"Exported {args.output} (outputs={output_count}{recurrent_note}, cg_lower_rad={args.cg_lower_rad}, "
        f"cg_upper_rad={args.cg_upper_rad}, i_max_a={args.i_max_a})"
    )
    try:
        max_error = validate(
            args.policy,
            args.output,
            args.obs_dim,
            args.samples,
            args.tolerance,
            args.cg_outputs,
            args.i_max_a,
            args.cg_lower_rad,
            args.cg_upper_rad,
        )
    except OnnxRuntimeUnavailable as exc:
        if args.require_validation:
            raise SystemExit(str(exc)) from exc
        print(f"WARNING: {exc}")
        print("Install with: python -m pip install onnxruntime")
    else:
        print(f"ONNX validation passed: max_error={max_error:.8g} < {args.tolerance:.8g}")


if __name__ == "__main__":
    main()
