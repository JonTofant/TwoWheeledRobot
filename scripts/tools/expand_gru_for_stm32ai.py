#!/usr/bin/env python3
"""Rewrite the paper GRU policy into an STM32AI-compatible one-step graph.

X-CUBE-AI 8.1 cannot import a native ONNX GRU that exposes ``h_out``.  The
firmware needs that state explicitly so it can thread it into the next 15 ms
inference and reset it between experiments.  This utility replaces the one
native GRU node with the mathematically equivalent Gemm/Sigmoid/Tanh/elementwise
operations while preserving the trained weights.

The converted graph uses flat state tensors ``h_in`` and ``h_out`` with shape
``[1, hidden_size]``.  This is the same contiguous hidden-state data as the
source graph's ``[1, 1, hidden_size]`` tensors.
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

import numpy as np


def _attribute(node, name: str, default):
	import onnx

	for attribute in node.attribute:
		if attribute.name == name:
			return onnx.helper.get_attribute_value(attribute)
	return default


def _tensor_shape(value_info) -> list[int | str]:
	shape: list[int | str] = []
	for dim in value_info.type.tensor_type.shape.dim:
		if dim.HasField("dim_value"):
			shape.append(dim.dim_value)
		else:
			shape.append(dim.dim_param)
	return shape


def _single_consumer(nodes, tensor_name: str):
	consumers = [node for node in nodes if tensor_name in node.input]
	return consumers[0] if len(consumers) == 1 else None


def convert(source: Path, output: Path) -> int:
	import onnx
	from onnx import TensorProto, helper, numpy_helper

	model = onnx.load(str(source))
	graph = model.graph
	gru_nodes = [node for node in graph.node if node.op_type == "GRU"]
	if len(gru_nodes) != 1:
		raise SystemExit(f"Expected exactly one GRU node, found {len(gru_nodes)}")
	if len(graph.input) != 2 or len(graph.output) != 2:
		raise SystemExit(
			f"Expected two inputs and two outputs, found {len(graph.input)} and {len(graph.output)}"
		)

	gru = gru_nodes[0]
	hidden_size = int(_attribute(gru, "hidden_size", 0))
	if hidden_size <= 0:
		raise SystemExit("GRU hidden_size is missing or invalid")
	if int(_attribute(gru, "linear_before_reset", 0)) != 1:
		raise SystemExit("Only PyTorch/ONNX GRU linear_before_reset=1 is supported")
	if _attribute(gru, "direction", b"forward") != b"forward":
		raise SystemExit("Only a single forward GRU is supported")
	if len(gru.input) < 6 or not gru.input[5]:
		raise SystemExit("The GRU must expose an explicit initial hidden-state input")
	if len(gru.output) < 2 or not gru.output[1]:
		raise SystemExit("The GRU must expose an explicit hidden-state output")

	obs_name = graph.input[0].name
	h_in_name = graph.input[1].name
	action_output = copy.deepcopy(graph.output[0])
	h_out_name = graph.output[1].name
	if gru.input[5] != h_in_name or gru.output[1] != h_out_name:
		raise SystemExit("GRU state input/output do not match the graph state interface")

	initializers = {item.name: numpy_helper.to_array(item) for item in graph.initializer}
	try:
		weights_input = initializers[gru.input[1]]
		weights_hidden = initializers[gru.input[2]]
		bias = initializers[gru.input[3]]
	except KeyError as exc:
		raise SystemExit(f"GRU parameter is not a constant initializer: {exc}") from exc

	expected_input_size = _tensor_shape(graph.input[0])[-1]
	if weights_input.shape != (1, 3 * hidden_size, expected_input_size):
		raise SystemExit(f"Unexpected GRU input-weight shape: {weights_input.shape}")
	if weights_hidden.shape != (1, 3 * hidden_size, hidden_size):
		raise SystemExit(f"Unexpected GRU recurrent-weight shape: {weights_hidden.shape}")
	if bias.shape != (1, 6 * hidden_size):
		raise SystemExit(f"Unexpected GRU bias shape: {bias.shape}")

	# Locate and remove the source exporter's obs Unsqueeze and the shape-only
	# path that turns GRU Y [seq,dir,batch,H] back into [batch,H].
	producer = next((node for node in graph.node if gru.input[0] in node.output), None)
	if producer is None or producer.op_type != "Unsqueeze" or producer.input[0] != obs_name:
		raise SystemExit("Expected the GRU input to be an Unsqueeze of the obs input")

	shape_nodes = []
	actor_input_name = gru.output[0]
	while True:
		consumer = _single_consumer(graph.node, actor_input_name)
		if consumer is None or consumer.op_type not in {"Identity", "Reshape", "Squeeze", "Unsqueeze"}:
			break
		shape_nodes.append(consumer)
		if len(consumer.output) != 1:
			raise SystemExit("Unexpected multi-output shape operation after GRU")
		actor_input_name = consumer.output[0]
	if not shape_nodes:
		raise SystemExit("Expected at least one shape operation after the GRU output")

	h = hidden_size
	w = weights_input[0]
	r = weights_hidden[0]
	b = bias[0]
	# ONNX GRU gate order is update, reset, hidden (z, r, h).  The bias stores
	# the three input biases followed by the three recurrent biases.
	parameter_arrays = {
		"stm32_gru_Wz": w[0:h],
		"stm32_gru_Wr": w[h : 2 * h],
		"stm32_gru_Wh": w[2 * h : 3 * h],
		"stm32_gru_Rz": r[0:h],
		"stm32_gru_Rr": r[h : 2 * h],
		"stm32_gru_Rh": r[2 * h : 3 * h],
		"stm32_gru_Wbz": b[0:h],
		"stm32_gru_Wbr": b[h : 2 * h],
		"stm32_gru_Wbh": b[2 * h : 3 * h],
		"stm32_gru_Rbz": b[3 * h : 4 * h],
		"stm32_gru_Rbr": b[4 * h : 5 * h],
		"stm32_gru_Rbh": b[5 * h : 6 * h],
		"stm32_gru_one": np.array(1.0, dtype=np.float32),
	}

	new_nodes = [
		helper.make_node("Gemm", [obs_name, "stm32_gru_Wz", "stm32_gru_Wbz"], ["stm32_gru_xz"], name="stm32_gru_xz", transB=1),
		helper.make_node("Gemm", [h_in_name, "stm32_gru_Rz", "stm32_gru_Rbz"], ["stm32_gru_hz"], name="stm32_gru_hz", transB=1),
		helper.make_node("Add", ["stm32_gru_xz", "stm32_gru_hz"], ["stm32_gru_z_pre"], name="stm32_gru_z_add"),
		helper.make_node("Sigmoid", ["stm32_gru_z_pre"], ["stm32_gru_z"], name="stm32_gru_z"),
		helper.make_node("Gemm", [obs_name, "stm32_gru_Wr", "stm32_gru_Wbr"], ["stm32_gru_xr"], name="stm32_gru_xr", transB=1),
		helper.make_node("Gemm", [h_in_name, "stm32_gru_Rr", "stm32_gru_Rbr"], ["stm32_gru_hr"], name="stm32_gru_hr", transB=1),
		helper.make_node("Add", ["stm32_gru_xr", "stm32_gru_hr"], ["stm32_gru_r_pre"], name="stm32_gru_r_add"),
		helper.make_node("Sigmoid", ["stm32_gru_r_pre"], ["stm32_gru_r"], name="stm32_gru_r"),
		helper.make_node("Gemm", [obs_name, "stm32_gru_Wh", "stm32_gru_Wbh"], ["stm32_gru_xh"], name="stm32_gru_xh", transB=1),
		helper.make_node("Gemm", [h_in_name, "stm32_gru_Rh", "stm32_gru_Rbh"], ["stm32_gru_hh"], name="stm32_gru_hh", transB=1),
		helper.make_node("Mul", ["stm32_gru_r", "stm32_gru_hh"], ["stm32_gru_reset_h"], name="stm32_gru_reset_h"),
		helper.make_node("Add", ["stm32_gru_xh", "stm32_gru_reset_h"], ["stm32_gru_n_pre"], name="stm32_gru_n_add"),
		helper.make_node("Tanh", ["stm32_gru_n_pre"], ["stm32_gru_n"], name="stm32_gru_n"),
		helper.make_node("Sub", ["stm32_gru_one", "stm32_gru_z"], ["stm32_gru_one_minus_z"], name="stm32_gru_one_minus_z"),
		helper.make_node("Mul", ["stm32_gru_one_minus_z", "stm32_gru_n"], ["stm32_gru_new_part"], name="stm32_gru_new_part"),
		helper.make_node("Mul", ["stm32_gru_z", h_in_name], ["stm32_gru_old_part"], name="stm32_gru_old_part"),
		helper.make_node("Add", ["stm32_gru_new_part", "stm32_gru_old_part"], [actor_input_name], name="stm32_gru_h_next"),
		helper.make_node("Identity", [actor_input_name], [h_out_name], name="stm32_gru_h_out"),
	]

	removed_ids = {id(gru), id(producer), *(id(node) for node in shape_nodes)}
	remaining_nodes = [copy.deepcopy(node) for node in graph.node if id(node) not in removed_ids]
	del graph.node[:]
	graph.node.extend(new_nodes + remaining_nodes)

	# Remove the packed native-GRU tensors and add the split gate tensors.
	packed_names = {gru.input[1], gru.input[2], gru.input[3]}
	kept_initializers = [copy.deepcopy(item) for item in graph.initializer if item.name not in packed_names]
	del graph.initializer[:]
	graph.initializer.extend(kept_initializers)
	graph.initializer.extend(
		numpy_helper.from_array(np.asarray(value, dtype=np.float32), name=name)
		for name, value in parameter_arrays.items()
	)

	# Remove orphan Constant nodes left behind by the deleted squeeze/unsqueeze
	# axes.  Repeat because removing one node can orphan another.
	changed = True
	while changed:
		used = {name for node in graph.node for name in node.input if name}
		used.update(item.name for item in graph.output)
		kept = [node for node in graph.node if node.op_type != "Constant" or any(name in used for name in node.output)]
		changed = len(kept) != len(graph.node)
		if changed:
			del graph.node[:]
			graph.node.extend(kept)

	# The two singleton dimensions in [1,1,H] contain no data.  Flattening the
	# explicit state avoids the STM32AI layout mapper that rejected [192].
	graph.input[1].CopyFrom(helper.make_tensor_value_info(h_in_name, TensorProto.FLOAT, [1, h]))
	del graph.output[:]
	graph.output.extend(
		[
			action_output,
			helper.make_tensor_value_info(h_out_name, TensorProto.FLOAT, [1, h]),
		]
	)

	onnx.checker.check_model(model)
	output.parent.mkdir(parents=True, exist_ok=True)
	onnx.save(model, str(output))
	return hidden_size


def validate_sequence(source: Path, converted: Path, hidden_size: int, steps: int, tolerance: float) -> tuple[float, float]:
	try:
		import onnxruntime as ort
	except ModuleNotFoundError as exc:
		raise SystemExit("onnxruntime is required for sequential equivalence validation") from exc

	options = ort.SessionOptions()
	options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
	reference = ort.InferenceSession(str(source), options, providers=["CPUExecutionProvider"])
	stm32 = ort.InferenceSession(str(converted), options, providers=["CPUExecutionProvider"])
	rng = np.random.default_rng(42)
	h_reference = np.zeros((1, 1, hidden_size), dtype=np.float32)
	h_stm32 = np.zeros((1, hidden_size), dtype=np.float32)
	max_action_error = 0.0
	max_state_error = 0.0

	for _ in range(steps):
		obs = np.clip(rng.standard_normal((1, 13)), -3.0, 3.0).astype(np.float32)
		action_reference, h_reference = reference.run(None, {"obs": obs, "h_in": h_reference})
		action_stm32, h_stm32 = stm32.run(None, {"obs": obs, "h_in": h_stm32})
		max_action_error = max(max_action_error, float(np.max(np.abs(action_reference - action_stm32))))
		max_state_error = max(
			max_state_error,
			float(np.max(np.abs(h_reference.reshape(1, hidden_size) - h_stm32))),
		)

	if max(max_action_error, max_state_error) >= tolerance:
		raise SystemExit(
			"Sequential validation failed: "
			f"action_error={max_action_error:.8g}, state_error={max_state_error:.8g}, tolerance={tolerance:.8g}"
		)
	return max_action_error, max_state_error


def main() -> None:
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("source", type=Path, help="Original ONNX model containing one native GRU")
	parser.add_argument("output", type=Path, help="Destination for the STM32AI-compatible ONNX model")
	parser.add_argument("--steps", type=int, default=256, help="Sequential equivalence-validation length")
	parser.add_argument("--tolerance", type=float, default=1.0e-4)
	args = parser.parse_args()

	if args.source.resolve() == args.output.resolve():
		raise SystemExit("Refusing to overwrite the original ONNX model")
	if not args.source.is_file():
		raise SystemExit(f"Source model not found: {args.source}")

	hidden_size = convert(args.source, args.output)
	action_error, state_error = validate_sequence(
		args.source, args.output, hidden_size, args.steps, args.tolerance
	)
	print(f"Wrote {args.output}")
	print(f"Hidden state: [1,{hidden_size}] ({hidden_size} float32 values)")
	print(
		f"Sequential validation passed ({args.steps} steps): "
		f"max_action_error={action_error:.8g}, max_state_error={state_error:.8g}"
	)


if __name__ == "__main__":
	main()
