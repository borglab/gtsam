#!/usr/bin/env python3
"""Drive time_bal.py over the large Dubrovnik BAL datasets and tabulate results.

Runs each dataset through the chosen PyPose/BAE backend (with GTSAM's
convergence rule, matching timeSFMBAL) and the GTSAM executable, repeating the
PyPose solve to report a robust (min/median) wall time. The GTSAM side reports
its own in-process solver time, so it is read once per dataset from the same
time_bal.py invocation.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
TIME_BAL = Path(__file__).resolve().parent / "time_bal.py"

DEFAULT_DATASETS = [
    "dubrovnik-16-22106-pre.txt",
    "dubrovnik-88-64298-pre.txt",
    "dubrovnik-135-90642-pre.txt",
]


def run_once(
    bal_file: Path,
    backend: str,
    device: str,
    steps: int,
    warmup_steps: int,
    relative_decrease_tol: float,
    gtsam_exe: Path | None,
) -> dict[str, Any]:
    # time_bal.py emits a CUDA/warp init banner on stdout, so read the result
    # from its JSON output file rather than parsing stdout.
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tmp:
        out_path = Path(tmp.name)
    try:
        cmd = [
            sys.executable,
            str(TIME_BAL),
            "--bal-file",
            str(bal_file),
            "--backend",
            backend,
            "--device",
            device,
            "--steps",
            str(steps),
            "--warmup-steps",
            str(warmup_steps),
            "--relative-decrease-tol",
            str(relative_decrease_tol),
            "--output-json",
            str(out_path),
        ]
        if gtsam_exe is not None:
            cmd += ["--gtsam-exe", str(gtsam_exe)]
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        return json.loads(out_path.read_text(encoding="utf-8"))
    finally:
        out_path.unlink(missing_ok=True)


def benchmark_dataset(
    bal_file: Path,
    backend: str,
    device: str,
    steps: int,
    warmup_steps: int,
    relative_decrease_tol: float,
    gtsam_exe: Path | None,
    repeats: int,
) -> dict[str, Any]:
    runs = [
        run_once(
            bal_file,
            backend,
            device,
            steps,
            warmup_steps,
            relative_decrease_tol,
            # Only ask GTSAM to run on the first repeat; it is deterministic
            # and its in-process timer does not need PyPose-style repeats.
            gtsam_exe if i == 0 else None,
        )
        for i in range(repeats)
    ]
    py_times = [r["pypose_ba"]["solve_loop_elapsed_s"] for r in runs]
    first = runs[0]
    summary: dict[str, Any] = {
        "dataset": bal_file.name,
        "backend": backend,
        "device": device,
        "n_cameras": first["n_cameras"],
        "n_points": first["n_points"],
        "n_observations": first["n_observations"],
        "python_iterations_run": first["pypose_ba"]["iterations_run"],
        "python_converged": first["pypose_ba"]["converged"],
        "python_final_error": first["pypose_ba"]["final_gtsam_style_error"],
        "python_time_min_s": min(py_times),
        "python_time_median_s": statistics.median(py_times),
        "python_times_s": py_times,
        "repeats": repeats,
    }
    if "final_error_comparison" in first:
        summary["gtsam_solvers"] = first["final_error_comparison"]["gtsam_solvers"]
    return summary


def format_table(rows: list[dict[str, Any]]) -> str:
    lines = []
    header = (
        "| Dataset | Obs | PyPose iters | PyPose s (min) | "
        "GTSAM Cholesky s | GTSAM Solver s | PyPose final | GTSAM final | rel.diff |"
    )
    sep = "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"
    lines.append(header)
    lines.append(sep)
    for r in rows:
        solvers = {s["solver"]: s for s in r.get("gtsam_solvers", [])}
        chol = solvers.get("MultifrontalCholesky")
        solv = solvers.get("MultifrontalSolver")
        chol_t = f"{chol['elapsed_s']:.3f}" if chol else "—"
        solv_t = f"{solv['elapsed_s']:.3f}" if solv else "—"
        ref = chol or solv
        gtsam_final = f"{ref['final_error']:.6g}" if ref else "—"
        reldiff = f"{ref['relative_diff']:.2%}" if ref else "—"
        lines.append(
            f"| {r['dataset'].replace('-pre.txt', '')} "
            f"| {r['n_observations']} "
            f"| {r['python_iterations_run']}{'*' if not r['python_converged'] else ''} "
            f"| {r['python_time_min_s']:.3f} "
            f"| {chol_t} | {solv_t} "
            f"| {r['python_final_error']:.6g} | {gtsam_final} | {reldiff} |"
        )
    lines.append("")
    lines.append("\\* = hit iteration cap without meeting the relative-decrease tolerance.")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-dir", type=Path, default=REPO_ROOT / "examples" / "Data"
    )
    parser.add_argument("--datasets", nargs="*", default=DEFAULT_DATASETS)
    parser.add_argument("--backend", choices=("pypose-sparse", "bae"), default="pypose-sparse")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--steps", type=int, default=50, help="LM iteration cap.")
    parser.add_argument("--warmup-steps", type=int, default=1)
    parser.add_argument("--relative-decrease-tol", type=float, default=0.01)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument(
        "--gtsam-exe", type=Path, default=REPO_ROOT / "build" / "timing" / "timeSFMBAL"
    )
    parser.add_argument("--output-json", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    gtsam_exe = args.gtsam_exe if args.gtsam_exe and args.gtsam_exe.is_file() else None
    if gtsam_exe is None:
        print(f"WARNING: GTSAM exe not found at {args.gtsam_exe}; skipping GTSAM side.")

    rows: list[dict[str, Any]] = []
    for name in args.datasets:
        bal_file = args.data_dir / name
        if not bal_file.is_file():
            print(f"WARNING: missing dataset {bal_file}, skipping.")
            continue
        print(f"\n=== {name} ({args.backend}, {args.repeats} repeats) ===", flush=True)
        row = benchmark_dataset(
            bal_file,
            args.backend,
            args.device,
            args.steps,
            args.warmup_steps,
            args.relative_decrease_tol,
            gtsam_exe,
            args.repeats,
        )
        rows.append(row)
        print(
            f"  PyPose: {row['python_iterations_run']} iters, "
            f"converged={row['python_converged']}, "
            f"{row['python_time_min_s']:.3f}s (min of {args.repeats}), "
            f"final={row['python_final_error']:.6g}",
            flush=True,
        )
        for s in row.get("gtsam_solvers", []):
            print(
                f"  GTSAM {s['solver']}: {s['iterations']} iters, "
                f"{s['elapsed_s']:.4f}s, final={s['final_error']:.6g}, "
                f"reldiff={s['relative_diff']:.2%}",
                flush=True,
            )

    print("\n" + format_table(rows))

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
        print(f"\nWrote {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
