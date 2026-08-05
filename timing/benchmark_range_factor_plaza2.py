#!/usr/bin/env python3
"""Run the Plaza2 RangeFactor benchmark and print a PR-friendly summary."""

from __future__ import annotations

import argparse
import csv
import statistics
import subprocess
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run timing/timeRangeFactorPlaza2 and summarize the results."
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=Path("build"),
        help="Build directory that contains timing/timeRangeFactorPlaza2.",
    )
    parser.add_argument(
        "--benchmark",
        type=Path,
        default=None,
        help="Optional explicit path to the benchmark executable.",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=1,
        help="Number of warmup runs to discard.",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=5,
        help="Number of measured runs.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("timing/results/range_factor_plaza2.csv"),
        help="CSV file written by the benchmark executable.",
    )
    parser.add_argument(
        "--no-robust",
        action="store_true",
        help="Disable the robust range noise model to match non-robust runs.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed forwarded to the benchmark executable.",
    )
    return parser.parse_args()


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def float_column(rows: list[dict[str, str]], key: str) -> list[float]:
    return [float(row[key]) for row in rows]


def main() -> int:
    args = parse_args()
    benchmark = (
        args.benchmark
        if args.benchmark is not None
        else args.build_dir / "timing" / "timeRangeFactorPlaza2"
    )
    if not benchmark.exists():
        raise SystemExit(f"Benchmark executable not found: {benchmark}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(benchmark),
        "--warmup",
        str(args.warmup),
        "--repeats",
        str(args.repeats),
        "--seed",
        str(args.seed),
        "--output",
        str(args.output),
    ]
    if args.no_robust:
        command.append("--no-robust")

    completed = subprocess.run(command, check=True, text=True, capture_output=True)
    print(completed.stdout, end="")

    rows = load_rows(args.output)
    if not rows:
        raise SystemExit(f"No measured rows written to {args.output}")

    solve_seconds = float_column(rows, "solve_seconds")
    total_seconds = float_column(rows, "total_seconds")
    batch_seconds = float_column(rows, "batch_initialization_seconds")
    update_seconds = float_column(rows, "update_seconds")
    estimate_seconds = float_column(rows, "calculate_estimate_seconds")

    shape = rows[0]
    print()
    print("PR summary")
    print(
        "Plaza2 range-only benchmark: "
        f"median solve {statistics.median(solve_seconds):.3f}s, "
        f"median total {statistics.median(total_seconds):.3f}s "
        f"across {len(rows)} measured runs."
    )
    print(
        "Per-run means: "
        f"batch init {statistics.mean(batch_seconds):.3f}s, "
        f"ISAM2 update {statistics.mean(update_seconds):.3f}s, "
        f"estimate {statistics.mean(estimate_seconds):.3f}s."
    )
    print(
        "Workload shape: "
        f"{shape['odometry_entries']} odometry entries, "
        f"{shape['range_factors_added']} range factors, "
        f"{shape['update_count']} updates, "
        f"{shape['initialized_landmarks']} landmarks."
    )
    print(f"CSV written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
