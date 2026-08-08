#!/usr/bin/env python3
"""
Reduce per-control-step protocol trace CSVs to one aggregate metrics row per run.

Purpose:
    Replace the hand reduction that produced the previous paper's comparison table.
    Reads a directory of trace CSVs written by scripts/protocol_bench.py (Isaac,
    MuJoCo or hardware -- the schema is identical) and writes a single metrics CSV
    matching metrics_per_run_template.csv plus the extra published columns.

Platform agnostic by construction:
    Nothing in this file branches on the ``platform`` column; it is grouped on and
    passed through only. The same invocation must reduce a simulation trace and a
    hardware trace. Missing optional columns (e.g. ``I_*_meas_A`` on simulation
    traces) are left blank rather than special-cased.

Input schema (scripts/protocol_bench.py, extending timeseries_log_template.csv):
    t_s,platform,scenario,run_id,initial_theta_deg,held,obs_0..obs_11,
    action_L,action_R,I_L_cmd_A,I_R_cmd_A,I_L_meas_A,I_R_meas_A

    Column sniffers accept explicit engineering-unit columns when present and fall
    back to the observation vector otherwise:
      pitch    <- pitch_deg | theta_deg | pitch_rad | theta | obs_2 * --pitch-scale-deg
      position <- x_m | base_x_m | pos_x_m | x | obs_0 * --pos-scale-m
      current  <- I_{L,R}_cmd_A | u_{left,right}_cmd_a | left_i_cmd_a/right_i_cmd_a

    OBSERVATION UNITS -- the one setting that can silently scale every angle:
    scripts/protocol_bench.py logs obs in RAW physical units (obs_2 = pitch in
    RADIANS, obs_0 = pos_err in METRES), so the defaults are
    --pitch-scale-deg 57.2957795 (degrees per radian) and --pos-scale-m 1.0.
    A trace that instead carries the network's NORMALISED observations must be
    reduced with ``--pitch-scale-deg 25 --pos-scale-m 0.5``. Getting this wrong
    scales every reported angle by 2.3x without producing an implausible number,
    so reduce_run() cross-checks the pitch at release against the independently
    recorded ``initial_theta_deg`` column and warns on a mismatch. Do not ignore
    that warning -- it is the only thing standing between a wrong scale and a
    published table.
    ``obs_0`` is position error, and the protocol scenarios hold a zero position
    target, so the position fallback is exact for them.

Grouping:
    One output row per (platform, scenario, run_id) tuple, gathered across every
    matched file. A trace file may hold one run or many; both work.

Windows:
    Rows with ``held == 1`` are the pre-release hold phase and are excluded from
    every metric. ``t_release`` is the time of the first non-held row; all metrics
    are computed over non-held rows only.
    ``theta_res_*`` use the final ``--residual-window-s`` seconds, selected as
    ``t >= t_end - window`` (inclusive, matching run_lqr_disturbance_sweep.py's
    last-1s window).

Metric definitions:
    max_abs_theta_deg  max |pitch| over the run.
    rms_theta_deg      sqrt(mean(pitch^2)) over the run.
    theta_res_bias_deg mean pitch over the final --residual-window-s seconds
                       (the residual lean; run_lqr_disturbance_sweep.py's
                       mean_pitch_last_1s_deg).
    theta_res_rms_deg  sqrt(mean(pitch^2)) over that same final window.
    x_final_m          position at the last non-held sample.
    rms_I_A            sqrt(mean(I_L^2 + I_R^2)) over the run, i.e. both wheels
                       combined -- the same formula as
                       run_step_disturbance_benchmark.py's
                       rms_current_command_a. For matched wheels this is sqrt(2)
                       times the per-wheel RMS; do not mix the two in one table.
    t_rec_s            recovery time, see below.

Recovery time (t_rec_s) -- the one metric with no canonical definition:
    Reported RELATIVE TO RELEASE (seconds after t_release, so an undisturbed run
    that never leaves the band reports 0.0).

    t_rec_s = t* - t_release, where t* is the earliest sample time at or after
    t_release such that |pitch| stays strictly below --rec-threshold-deg for every
    consecutive sample from t* through at least t* + --rec-window-s.

    Consequences, stated so the number can be audited:
      * It is a "settled for a window" definition, not a "never exceeds again"
        definition. A run that settles, then is knocked out of the band later,
        still reports the first settling. Compare max_abs_theta_deg and
        theta_res_rms_deg to catch that case.
      * If the trace ends less than --rec-window-s after the first in-band sample,
        there is not enough evidence to confirm settling and the cell is left
        BLANK rather than guessed. Blank means "not demonstrated", not "instant".
      * The band test is strict (<), the sample grid is not interpolated, so t_rec
        is quantised up to the next control step.

Edit here when:
    The trace schema gains columns, or the recovery-time convention changes. If
    the convention changes, update the paper caption in the same commit.

Avoid changing here without also checking:
    metrics_per_run_template.csv (column order), scripts/protocol_bench.py (the
    writer of these traces), and any table-generation script that consumes the
    output.

Usage:
    python3 scripts/reduce_protocol_metrics.py outputs/protocol_traces \\
        --out outputs/metrics_per_run.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import OrderedDict
from pathlib import Path

# Observation units. scripts/protocol_bench.py logs RAW physical units, so obs_2
# is pitch in radians and obs_0 is pos_err in metres. See the module docstring.
DEFAULT_PITCH_SCALE_DEG = 180.0 / math.pi
DEFAULT_POS_SCALE_M = 1.0

# Pitch at release should equal the recorded initial_theta_deg. Warn beyond this.
THETA0_CHECK_MIN_DEG = 0.5
THETA0_CHECK_REL_TOL = 0.2
THETA0_CHECK_ABS_TOL_DEG = 1.0

DEFAULT_REC_THRESHOLD_DEG = 2.0
DEFAULT_REC_WINDOW_S = 1.0
DEFAULT_RESIDUAL_WINDOW_S = 2.0

KEY_COLUMNS = ("platform", "scenario", "run_id")

OUTPUT_COLUMNS = [
    # metrics_per_run_template.csv schema, in order.
    "platform",
    "scenario",
    "run_id",
    "initial_theta_deg",
    "max_abs_theta_deg",
    "rms_theta_deg",
    "rms_I_A",
    "t_rec_s",
    "x_final_m",
    # Additional published columns.
    "theta_res_bias_deg",
    "theta_res_rms_deg",
    # Provenance, so an odd number can be traced back without re-deriving it.
    "n_samples",
    "t_release_s",
    "t_end_s",
    "source_files",
]

TIME_COLUMNS = ("t_s", "time_s", "time", "t")
PITCH_DEG_COLUMNS = ("pitch_deg", "theta_deg")
PITCH_RAD_COLUMNS = ("pitch_rad", "theta_rad", "theta")
POSITION_COLUMNS = ("x_m", "base_x_m", "pos_x_m", "final_base_x_m", "x")
CURRENT_COLUMN_PAIRS = (
    ("I_L_cmd_A", "I_R_cmd_A"),
    ("i_l_cmd_a", "i_r_cmd_a"),
    ("u_left_cmd_a", "u_right_cmd_a"),
    ("left_i_cmd_a", "right_i_cmd_a"),
    ("left_i_des_a", "right_i_des_a"),
    ("i_cmd_left", "i_cmd_right"),
)


class TraceSchemaError(RuntimeError):
    """Raised when a trace CSV lacks a column no metric can be computed without."""


def to_float(value: object) -> float:
    """Parse a CSV cell to float; blank, None and unparseable become NaN."""
    if value is None:
        return math.nan
    text = str(value).strip()
    if not text:
        return math.nan
    try:
        return float(text)
    except ValueError:
        return math.nan


def first_present(fieldnames: list[str], candidates: tuple[str, ...]) -> str | None:
    lookup = {name.lower(): name for name in fieldnames}
    for candidate in candidates:
        if candidate in lookup:
            return lookup[candidate]
        if candidate.lower() in lookup:
            return lookup[candidate.lower()]
    return None


def first_present_pair(fieldnames: list[str], pairs: tuple[tuple[str, str], ...]) -> tuple[str, str] | None:
    lookup = {name.lower(): name for name in fieldnames}
    for left, right in pairs:
        if left.lower() in lookup and right.lower() in lookup:
            return lookup[left.lower()], lookup[right.lower()]
    return None


def time_column(fieldnames: list[str]) -> str:
    column = first_present(fieldnames, TIME_COLUMNS)
    if column is None:
        raise TraceSchemaError(f"no time column among {TIME_COLUMNS}")
    return column


def pitch_reader(fieldnames: list[str], pitch_scale_deg: float):
    """Return a row -> pitch-in-degrees callable, preferring explicit columns."""
    column = first_present(fieldnames, PITCH_DEG_COLUMNS)
    if column is not None:
        return lambda row: to_float(row.get(column))
    column = first_present(fieldnames, PITCH_RAD_COLUMNS)
    if column is not None:
        return lambda row: math.degrees(to_float(row.get(column)))
    column = first_present(fieldnames, ("obs_2",))
    if column is not None:
        return lambda row: to_float(row.get(column)) * pitch_scale_deg
    raise TraceSchemaError("no pitch column among pitch_deg, theta_deg, pitch_rad, theta, obs_2")


def position_reader(fieldnames: list[str], pos_scale_m: float):
    """Return a row -> position-in-metres callable, or None if unavailable."""
    column = first_present(fieldnames, POSITION_COLUMNS)
    if column is not None:
        return lambda row: to_float(row.get(column))
    column = first_present(fieldnames, ("obs_0",))
    if column is not None:
        # obs_0 is position error normalised by pos_scale_m; the protocol
        # scenarios hold a zero position target, so error == position.
        return lambda row: to_float(row.get(column)) * pos_scale_m
    return None


def current_reader(fieldnames: list[str]):
    """Return a row -> (I_left, I_right) callable, or None if unavailable."""
    pair = first_present_pair(fieldnames, CURRENT_COLUMN_PAIRS)
    if pair is None:
        return None
    left, right = pair
    return lambda row: (to_float(row.get(left)), to_float(row.get(right)))


def held_reader(fieldnames: list[str]):
    """Return a row -> bool callable that is True during the pre-release hold."""
    column = first_present(fieldnames, ("held",))
    if column is None:
        return lambda row: False

    def is_held(row: dict) -> bool:
        raw = str(row.get(column, "")).strip().lower()
        if raw in ("", "0", "false", "no", "nan"):
            return False
        if raw in ("1", "true", "yes"):
            return True
        value = to_float(raw)
        return bool(value == value and value != 0.0)

    return is_held


def rms(values: list[float]) -> float:
    if not values:
        return math.nan
    return math.sqrt(sum(v * v for v in values) / len(values))


def mean(values: list[float]) -> float:
    if not values:
        return math.nan
    return sum(values) / len(values)


def peak_abs(values: list[float]) -> float:
    if not values:
        return math.nan
    return max(abs(v) for v in values)


def recovery_time_s(times: list[float], pitch: list[float], threshold_deg: float, window_s: float) -> float:
    """First settling time relative to times[0]; NaN when not demonstrated.

    See the module docstring for the full definition. Scans for the earliest
    maximal run of consecutive in-band samples that spans at least window_s.
    """
    if not times:
        return math.nan
    t_end = times[-1]
    index = 0
    count = len(times)
    while index < count:
        if abs(pitch[index]) >= threshold_deg:
            index += 1
            continue
        start = index
        while index < count and abs(pitch[index]) < threshold_deg:
            index += 1
        span_end = times[index - 1]
        if span_end - times[start] >= window_s:
            return times[start] - times[0]
        if index >= count and t_end - times[start] >= window_s:
            # Unreachable in practice (span_end == t_end here) but keeps the
            # "ran out of trace" case explicit.
            return times[start] - times[0]
    return math.nan


def read_trace(path: Path, pitch_scale_deg: float, pos_scale_m: float) -> list[dict]:
    """Read one trace CSV into normalised sample dicts (one per row)."""
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        if not fieldnames:
            raise TraceSchemaError("file has no header row")
        t_col = time_column(fieldnames)
        read_pitch = pitch_reader(fieldnames, pitch_scale_deg)
        read_position = position_reader(fieldnames, pos_scale_m)
        read_current = current_reader(fieldnames)
        is_held = held_reader(fieldnames)
        has_key = {name: first_present(fieldnames, (name,)) for name in KEY_COLUMNS}
        theta0_col = first_present(fieldnames, ("initial_theta_deg",))

        samples = []
        for row in reader:
            if row is None:
                continue
            t = to_float(row.get(t_col))
            pitch = read_pitch(row)
            if math.isnan(t) or math.isnan(pitch):
                continue
            currents = read_current(row) if read_current is not None else (math.nan, math.nan)
            samples.append(
                {
                    "key": tuple(
                        str(row.get(has_key[name], "")).strip() if has_key[name] else "" for name in KEY_COLUMNS
                    ),
                    "t": t,
                    "pitch": pitch,
                    "held": is_held(row),
                    "position": read_position(row) if read_position is not None else math.nan,
                    "i_left": currents[0],
                    "i_right": currents[1],
                    "initial_theta_deg": (str(row.get(theta0_col, "")).strip() if theta0_col else ""),
                    "source": path.name,
                }
            )
    return samples


def fmt(value: float, digits: int = 9) -> str:
    """Format a metric for CSV; NaN/inf become blank. 9 significant digits keeps
    this intermediate file lossless enough to re-check by hand -- round for the
    paper table downstream, not here."""
    if value is None:
        return ""
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return ""
    return f"{value:.{digits}g}"


def reduce_run(key: tuple[str, str, str], samples: list[dict], args: argparse.Namespace) -> dict:
    """Reduce one (platform, scenario, run_id) group to a metrics row."""
    samples = sorted(samples, key=lambda s: s["t"])
    initial_theta = next((s["initial_theta_deg"] for s in samples if s["initial_theta_deg"]), "")
    sources = sorted({s["source"] for s in samples})
    row = dict.fromkeys(OUTPUT_COLUMNS, "")
    row["platform"], row["scenario"], row["run_id"] = key
    row["initial_theta_deg"] = initial_theta
    row["source_files"] = ";".join(sources)

    run = [s for s in samples if not s["held"]]
    row["n_samples"] = str(len(run))
    if not run:
        print(f"warning: {key} has no non-held samples; metrics left blank", file=sys.stderr)
        return row

    times = [s["t"] for s in run]
    pitch = [s["pitch"] for s in run]
    t_release = times[0]
    t_end = times[-1]
    row["t_release_s"] = fmt(t_release)
    row["t_end_s"] = fmt(t_end)

    row["max_abs_theta_deg"] = fmt(peak_abs(pitch))
    row["rms_theta_deg"] = fmt(rms(pitch))

    residual = [p for t, p in zip(times, pitch) if t >= t_end - args.residual_window_s]
    row["theta_res_bias_deg"] = fmt(mean(residual))
    row["theta_res_rms_deg"] = fmt(rms(residual))

    currents = [(s["i_left"], s["i_right"]) for s in run]
    usable = [(left, right) for left, right in currents if not (math.isnan(left) or math.isnan(right))]
    if usable:
        row["rms_I_A"] = fmt(math.sqrt(sum(left * left + right * right for left, right in usable) / len(usable)))

    positions = [s["position"] for s in run if not math.isnan(s["position"])]
    if positions:
        row["x_final_m"] = fmt(positions[-1])

    row["t_rec_s"] = fmt(recovery_time_s(times, pitch, args.rec_threshold_deg, args.rec_window_s))
    warn_theta0_mismatch(key, initial_theta, pitch[0])
    return row


def warn_theta0_mismatch(key: tuple[str, str, str], initial_theta: str, pitch_at_release: float) -> None:
    """Cross-check pitch at release against the recorded initial_theta_deg.

    initial_theta_deg is written by the logger independently of the observation
    scaling, so a mismatch is the signature of a wrong --pitch-scale-deg (e.g.
    reducing radian-valued obs_2 with the normalised 25 deg scale).
    """
    theta0 = abs(to_float(initial_theta))
    if math.isnan(theta0) or theta0 < THETA0_CHECK_MIN_DEG:
        return
    observed = abs(pitch_at_release)
    tolerance = max(THETA0_CHECK_ABS_TOL_DEG, THETA0_CHECK_REL_TOL * theta0)
    if abs(observed - theta0) > tolerance:
        print(
            f"warning: {key} pitch at release is {observed:.3f} deg but initial_theta_deg is "
            f"{theta0:.3f} deg -- check --pitch-scale-deg (default assumes obs_2 is in radians; "
            f"normalised traces need 25)",
            file=sys.stderr,
        )


def collect_traces(trace_dir: Path, pattern: str, recursive: bool, out_path: Path) -> list[Path]:
    paths = sorted(trace_dir.rglob(pattern) if recursive else trace_dir.glob(pattern))
    resolved_out = out_path.resolve()
    return [p for p in paths if p.is_file() and p.resolve() != resolved_out]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reduce protocol trace CSVs to one aggregate metrics row per run.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("trace_dir", type=Path, help="Directory containing trace CSVs.")
    parser.add_argument("--out", type=Path, required=True, help="Output metrics CSV path.")
    parser.add_argument("--pattern", default="*.csv", help="Glob pattern for trace files.")
    parser.add_argument("--recursive", action="store_true", help="Recurse into subdirectories.")
    parser.add_argument(
        "--rec-threshold-deg",
        type=float,
        default=DEFAULT_REC_THRESHOLD_DEG,
        help="Recovery band: |pitch| must stay strictly below this.",
    )
    parser.add_argument(
        "--rec-window-s",
        type=float,
        default=DEFAULT_REC_WINDOW_S,
        help="Recovery settling window the band must be held for.",
    )
    parser.add_argument(
        "--residual-window-s",
        type=float,
        default=DEFAULT_RESIDUAL_WINDOW_S,
        help="Trailing window for theta_res_bias_deg and theta_res_rms_deg.",
    )
    parser.add_argument(
        "--pitch-scale-deg",
        type=float,
        default=DEFAULT_PITCH_SCALE_DEG,
        help="Degrees per unit of obs_2 (default = deg/rad, matching protocol_bench.py's "
        "raw units). Use 25 for normalised-observation traces.",
    )
    parser.add_argument(
        "--pos-scale-m",
        type=float,
        default=DEFAULT_POS_SCALE_M,
        help="Metres per unit of obs_0 (default = raw metres). Use 0.5 for normalised traces.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.trace_dir.is_dir():
        print(f"error: not a directory: {args.trace_dir}", file=sys.stderr)
        return 2

    paths = collect_traces(args.trace_dir, args.pattern, args.recursive, args.out)
    if not paths:
        print(f"error: no files matching {args.pattern!r} in {args.trace_dir}", file=sys.stderr)
        return 2

    groups: OrderedDict[tuple[str, str, str], list[dict]] = OrderedDict()
    for path in paths:
        try:
            samples = read_trace(path, args.pitch_scale_deg, args.pos_scale_m)
        except TraceSchemaError as exc:
            print(f"warning: skipping {path}: {exc}", file=sys.stderr)
            continue
        if not samples:
            print(f"warning: skipping {path}: no usable rows", file=sys.stderr)
            continue
        for sample in samples:
            groups.setdefault(sample["key"], []).append(sample)

    if not groups:
        print("error: no usable trace rows found", file=sys.stderr)
        return 2

    rows = [reduce_run(key, samples, args) for key, samples in sorted(groups.items())]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"wrote {len(rows)} run(s) from {len(paths)} file(s) to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
