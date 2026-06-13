#!/usr/bin/env python3
"""Compare PyPose/BAE sparse bundle adjustment timing with GTSAM timeSFMBAL."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import re
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any

import torch
import torch.nn as nn


DTYPE = torch.float64
INITIAL_ERROR_RE = re.compile(r"Initial error:\s*([0-9.eE+-]+)")
# Matches the per-solver result pair printed by timeSFMBAL:
#   MultifrontalSolver: 0.0059 s
# Final error: 0.0560241721432198, iterations: 8
FINAL_RESULT_RE = re.compile(
    r"^\s*(?P<label>\w+):\s*(?P<elapsed>[0-9.eE+-]+) s\s*\n"
    r"Final error:\s*(?P<error>[0-9.eE+-]+), iterations:\s*(?P<iterations>\d+)",
    re.MULTILINE,
)
# GTSAM lambdaInitial from LevenbergMarquardtParams::SetCeresDefaults.
GTSAM_LAMBDA_INITIAL = 1e-4


def rotvec_to_quat_xyzw(rotvec: torch.Tensor) -> torch.Tensor:
    theta = torch.linalg.norm(rotvec, dim=-1, keepdim=True)
    half_theta = 0.5 * theta
    cos_half = torch.cos(half_theta)
    sin_half = torch.sin(half_theta)
    scale = torch.where(theta > 1e-12, sin_half / theta, 0.5 - theta.square() / 48.0)
    return torch.cat([rotvec * scale, cos_half], dim=-1)


def read_bal(path: Path, use_quat: bool = True) -> dict[str, Any]:
    tokens = path.read_text(encoding="utf-8").split()
    cursor = 0

    def next_token() -> str:
        nonlocal cursor
        if cursor >= len(tokens):
            raise ValueError(f"Unexpected end of BAL file while reading {path}")
        token = tokens[cursor]
        cursor += 1
        return token

    n_cameras = int(next_token())
    n_points = int(next_token())
    n_observations = int(next_token())

    cidx = torch.empty(n_observations, dtype=torch.long)
    pidx = torch.empty(n_observations, dtype=torch.long)
    points_2d = torch.empty((n_observations, 2), dtype=DTYPE)
    for obs in range(n_observations):
        cidx[obs] = int(next_token())
        pidx[obs] = int(next_token())
        points_2d[obs, 0] = float(next_token())
        points_2d[obs, 1] = float(next_token())

    camera_params = torch.empty((n_cameras, 9), dtype=DTYPE)
    for camera in range(n_cameras):
        for col in range(9):
            camera_params[camera, col] = float(next_token())

    points_3d = torch.empty((n_points, 3), dtype=DTYPE)
    for point in range(n_points):
        for col in range(3):
            points_3d[point, col] = float(next_token())

    observation_counts = torch.bincount(pidx, minlength=n_points)
    keep_observation = observation_counts[pidx] >= 2
    n_observations_raw = n_observations
    n_dropped_observations = int((~keep_observation).sum().item())
    n_dropped_tracks = int((observation_counts < 2).sum().item())
    cidx = cidx[keep_observation]
    pidx = pidx[keep_observation]
    points_2d = points_2d[keep_observation]
    n_observations = int(points_2d.shape[0])

    if use_quat:
        q = rotvec_to_quat_xyzw(camera_params[:, :3])
        camera_params = torch.cat([camera_params[:, 3:6], q, camera_params[:, 6:]], dim=1)
    else:
        camera_params = torch.cat(
            [camera_params[:, 3:6], camera_params[:, :3], camera_params[:, 6:]], dim=1
        )

    return {
        "problem_name": path.stem,
        "n_cameras": n_cameras,
        "n_points": n_points,
        "n_observations": n_observations,
        "n_observations_raw": n_observations_raw,
        "n_dropped_observations": n_dropped_observations,
        "n_dropped_tracks": n_dropped_tracks,
        "camera_params": camera_params,
        "points_3d": points_3d,
        "points_2d": points_2d,
        "cidx": cidx,
        "pidx": pidx,
    }


def sync_device(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def make_model_class():
    import pypose as pp
    from pypose.autograd.function import psjac

    @psjac
    def project(
        points: torch.Tensor, poses: torch.Tensor, intrinsics: torch.Tensor
    ) -> torch.Tensor:
        projection = poses.Act(points)
        projection = -projection[..., :2] / projection[..., [2]]

        f = intrinsics[..., [0]]
        k1 = intrinsics[..., [1]]
        k2 = intrinsics[..., [2]]
        n = torch.sum(projection.square(), dim=-1, keepdim=True)
        radial = 1 + k1 * n + k2 * n.square()
        return projection * radial * f

    class ReprojectionResidual(nn.Module):
        def __init__(self, camera_params: torch.Tensor, points: torch.Tensor) -> None:
            super().__init__()
            self.pose = pp.Parameter(pp.SE3(camera_params[:, :7]), sjac=True)
            self.intrinsics = pp.Parameter(camera_params[:, 7:], sjac=True)
            self.points = pp.Parameter(points, sjac=True)

        def forward(
            self, observes: torch.Tensor, cidx: torch.Tensor, pidx: torch.Tensor
        ) -> torch.Tensor:
            return project(self.points[pidx], self.pose[cidx], self.intrinsics[cidx]) - observes

    return ReprojectionResidual


@torch.no_grad()
def objective_sumsq(
    model: nn.Module, inputs: tuple[torch.Tensor, torch.Tensor, torch.Tensor]
) -> float:
    residual = model(*inputs)
    return float(torch.sum(residual.square()).detach().cpu())


def objective_metrics(
    model: nn.Module, inputs: tuple[torch.Tensor, torch.Tensor, torch.Tensor]
) -> dict[str, float]:
    sumsq = objective_sumsq(model, inputs)
    return {
        "sumsq": sumsq,
        "gtsam_style_error": 0.5 * sumsq,
    }


def loss_to_float(loss: torch.Tensor) -> float:
    return float(loss.detach().cpu())


def run_lm_loop(
    optimizer: Any,
    step_input: Any,
    initial_sumsq: float,
    steps: int,
    relative_decrease_tol: float | None,
    record_losses: bool,
    device: torch.device,
) -> dict[str, Any]:
    """Timed LM loop, optionally with GTSAM's relative-decrease stopping rule.

    GTSAM stops when (currentError - newError) / currentError <= tol (see
    checkConvergence in NonlinearOptimizer.cpp). Relative decrease is
    scale-invariant, so checking it on the raw sum-of-squares loss matches
    checking it on GTSAM's 0.5*sumsq error. The check needs the scalar loss on
    the CPU each step; GTSAM evaluates its error every iteration too, so this
    sync is part of comparable work, not overhead.
    """
    losses: list[float] = []
    iterations_run = 0
    converged = False
    last_loss = initial_sumsq
    sync_device(device)
    start = time.perf_counter()
    for _ in range(steps):
        loss = optimizer.step(step_input)
        iterations_run += 1
        if relative_decrease_tol is not None:
            loss_value = loss_to_float(loss)
            if record_losses:
                losses.append(loss_value)
            if last_loss - loss_value <= relative_decrease_tol * last_loss:
                converged = True
                break
            last_loss = loss_value
        elif record_losses:
            losses.append(loss_to_float(loss))
    sync_device(device)
    elapsed_s = time.perf_counter() - start
    return {
        "elapsed_s": elapsed_s,
        "iterations_run": iterations_run,
        "converged": converged,
        "losses": losses,
    }


def run_pypose_sparse(
    dataset: dict[str, Any],
    device: torch.device,
    steps: int,
    warmup_steps: int,
    pcg_tol: float,
    pcg_maxiter: int,
    initial_damping: float,
    relative_decrease_tol: float | None,
    record_losses: bool,
) -> dict[str, Any]:
    import pypose as pp
    from pypose.optim import LM
    from pypose.optim.solver import PCG
    from pypose.optim.strategy import TrustRegion

    ReprojectionResidual = make_model_class()

    tensors = {
        key: value.to(device)
        for key, value in dataset.items()
        if isinstance(value, torch.Tensor)
    }
    def make_run_state():
        inputs = (tensors["points_2d"], tensors["cidx"], tensors["pidx"])
        model = ReprojectionResidual(
            tensors["camera_params"].clone(), tensors["points_3d"].clone()
        ).to(device)
        solver = PCG(tol=pcg_tol, maxiter=pcg_maxiter)
        # TrustRegion sets initial damping = 1/radius, and applies it as
        # diag *= (1 + damping) -- the same multiplicative diagonal damping
        # GTSAM uses, so initial_damping corresponds to GTSAM's lambdaInitial.
        strategy = TrustRegion(radius=1.0 / initial_damping, up=2.0, down=0.5**4)
        try:
            optimizer = LM(model, solver=solver, strategy=strategy, sparse=True)
        except TypeError:
            optimizer = LM(model, solver, strategy, sparse=True)
        return inputs, model, optimizer

    if warmup_steps:
        warmup_inputs, warmup_model, warmup_optimizer = make_run_state()
        for _ in range(warmup_steps):
            warmup_optimizer.step(warmup_inputs)
        sync_device(device)
        del warmup_inputs, warmup_model, warmup_optimizer
        if device.type == "cuda":
            torch.cuda.empty_cache()

    inputs, model, optimizer = make_run_state()

    initial_objective = objective_metrics(model, inputs)
    loop = run_lm_loop(
        optimizer,
        inputs,
        initial_objective["sumsq"],
        steps,
        relative_decrease_tol,
        record_losses,
        device,
    )
    final_objective = objective_metrics(model, inputs)

    return {
        "backend": "pypose-sparse",
        "device": str(device),
        "steps": steps,
        "iterations_run": loop["iterations_run"],
        "converged": loop["converged"],
        "relative_decrease_tol": relative_decrease_tol,
        "warmup_steps": warmup_steps,
        "initial_damping": initial_damping,
        "record_losses": record_losses,
        "solve_loop_elapsed_s": loop["elapsed_s"],
        "solve_loop_seconds_per_step": (
            loop["elapsed_s"] / loop["iterations_run"]
            if loop["iterations_run"]
            else math.nan
        ),
        "initial_sumsq": initial_objective["sumsq"],
        "initial_gtsam_style_error": initial_objective["gtsam_style_error"],
        "final_sumsq": final_objective["sumsq"],
        "final_gtsam_style_error": final_objective["gtsam_style_error"],
        "reported_loss_sumsq": loop["losses"],
    }


def run_bae(
    dataset: dict[str, Any],
    device: torch.device,
    steps: int,
    warmup_steps: int,
    pcg_tol: float,
    pcg_maxiter: int,
    initial_damping: float,
    relative_decrease_tol: float | None,
    reject: int,
    record_losses: bool,
) -> dict[str, Any]:
    import pypose as pp
    from bae.optim import LM
    from bae.utils.pysolvers import PCG

    ReprojectionResidual = make_model_class()

    tensors = {
        key: value.to(device)
        for key, value in dataset.items()
        if isinstance(value, torch.Tensor)
    }
    inputs_dict = {
        "observes": tensors["points_2d"],
        "cidx": tensors["cidx"],
        "pidx": tensors["pidx"],
    }

    def make_run_state():
        inputs_tuple = (tensors["points_2d"], tensors["cidx"], tensors["pidx"])
        model = ReprojectionResidual(
            tensors["camera_params"].clone(), tensors["points_3d"].clone()
        ).to(device)
        solver = PCG(tol=pcg_tol, maxiter=pcg_maxiter)
        strategy = pp.optim.strategy.TrustRegion(
            radius=1.0 / initial_damping, up=2.0, down=0.5**4
        )
        optimizer = LM(model, strategy=strategy, solver=solver, reject=reject)
        return inputs_tuple, model, optimizer

    if warmup_steps:
        _, warmup_model, warmup_optimizer = make_run_state()
        for _ in range(warmup_steps):
            warmup_optimizer.step(inputs_dict)
        sync_device(device)
        del warmup_model, warmup_optimizer
        if device.type == "cuda":
            torch.cuda.empty_cache()

    inputs_tuple, model, optimizer = make_run_state()

    initial_objective = objective_metrics(model, inputs_tuple)
    loop = run_lm_loop(
        optimizer,
        inputs_dict,
        initial_objective["sumsq"],
        steps,
        relative_decrease_tol,
        record_losses,
        device,
    )
    final_objective = objective_metrics(model, inputs_tuple)

    return {
        "backend": "bae",
        "device": str(device),
        "steps": steps,
        "iterations_run": loop["iterations_run"],
        "converged": loop["converged"],
        "relative_decrease_tol": relative_decrease_tol,
        "warmup_steps": warmup_steps,
        "initial_damping": initial_damping,
        "record_losses": record_losses,
        "solve_loop_elapsed_s": loop["elapsed_s"],
        "solve_loop_seconds_per_step": (
            loop["elapsed_s"] / loop["iterations_run"]
            if loop["iterations_run"]
            else math.nan
        ),
        "initial_sumsq": initial_objective["sumsq"],
        "initial_gtsam_style_error": initial_objective["gtsam_style_error"],
        "final_sumsq": final_objective["sumsq"],
        "final_gtsam_style_error": final_objective["gtsam_style_error"],
        "reported_loss_sumsq": loop["losses"],
    }


def parse_gtsam_initial_errors(stdout: str) -> list[float]:
    return [float(match.group(1)) for match in INITIAL_ERROR_RE.finditer(stdout)]


def parse_gtsam_final_results(stdout: str) -> list[dict[str, Any]]:
    return [
        {
            "solver": match.group("label"),
            "elapsed_s": float(match.group("elapsed")),
            "final_error": float(match.group("error")),
            "iterations": int(match.group("iterations")),
        }
        for match in FINAL_RESULT_RE.finditer(stdout)
    ]


def environment_info(device: torch.device) -> dict[str, Any]:
    info: dict[str, Any] = {
        "platform": platform.platform(),
        "processor": platform.processor(),
        "cpu_count": os.cpu_count(),
        "torch_num_threads": torch.get_num_threads(),
        "torch_num_interop_threads": torch.get_num_interop_threads(),
        "omp_num_threads": os.environ.get("OMP_NUM_THREADS"),
        "mkl_num_threads": os.environ.get("MKL_NUM_THREADS"),
    }
    if device.type == "cuda" and torch.cuda.is_available():
        props = torch.cuda.get_device_properties(device)
        info["cuda_device_name"] = props.name
        info["cuda_device_total_memory_bytes"] = props.total_memory
        info["cuda_device_capability"] = f"{props.major}.{props.minor}"
    return info


def run_gtsam(gtsam_exe: Path, bal_file: Path) -> dict[str, Any]:
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tmp:
        json_path = Path(tmp.name)
    try:
        cmd = [str(gtsam_exe), "--benchmark-action-json", str(json_path), str(bal_file)]
        start = time.perf_counter()
        proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
        elapsed_s = time.perf_counter() - start
        with json_path.open("r", encoding="utf-8") as f:
            benchmark_entries = json.load(f)
        initial_errors = parse_gtsam_initial_errors(proc.stdout)
        final_results = parse_gtsam_final_results(proc.stdout)
        return {
            "command": cmd,
            "process_elapsed_s_not_for_solver_comparison": elapsed_s,
            "benchmark_entries": benchmark_entries,
            "initial_errors_from_stdout": initial_errors,
            "final_results_from_stdout": final_results,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
        }
    finally:
        json_path.unlink(missing_ok=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bal-file", type=Path, required=True)
    parser.add_argument("--backend", choices=("pypose-sparse", "bae"), default="pypose-sparse")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument(
        "--warmup-steps",
        type=int,
        default=0,
        help="Untimed optimizer steps on a separate model before the timed solve.",
    )
    parser.add_argument("--pcg-tol", type=float, default=1e-4)
    parser.add_argument("--pcg-maxiter", type=int, default=250)
    parser.add_argument(
        "--initial-damping",
        type=float,
        default=GTSAM_LAMBDA_INITIAL,
        help="Initial LM damping; default matches GTSAM's Ceres-style lambdaInitial.",
    )
    parser.add_argument(
        "--relative-decrease-tol",
        type=float,
        default=0.01,
        help="Stop when the relative loss decrease per step falls to this value, "
        "matching timeSFMBAL's setRelativeErrorTol(0.01). --steps becomes the "
        "iteration cap. Pass a negative value to disable and always run --steps "
        "fixed iterations.",
    )
    parser.add_argument("--reject", type=int, default=30, help="Standalone BAE LM reject limit.")
    parser.add_argument("--gtsam-exe", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--print-gtsam-stdout", action="store_true")
    parser.add_argument(
        "--record-losses",
        action="store_true",
        help="Record per-step loss values. This copies loss tensors to CPU inside the timed loop.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.bal_file.is_file():
        raise FileNotFoundError(args.bal_file)

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested, but torch.cuda.is_available() is false.")
    if args.steps < 0:
        raise ValueError("--steps must be non-negative")
    if args.warmup_steps < 0:
        raise ValueError("--warmup-steps must be non-negative")

    torch.set_default_dtype(DTYPE)
    dataset = read_bal(args.bal_file, use_quat=True)

    relative_decrease_tol = (
        args.relative_decrease_tol if args.relative_decrease_tol >= 0 else None
    )
    if args.backend == "pypose-sparse":
        pypose_result = run_pypose_sparse(
            dataset,
            device,
            args.steps,
            args.warmup_steps,
            args.pcg_tol,
            args.pcg_maxiter,
            args.initial_damping,
            relative_decrease_tol,
            args.record_losses,
        )
    else:
        pypose_result = run_bae(
            dataset,
            device,
            args.steps,
            args.warmup_steps,
            args.pcg_tol,
            args.pcg_maxiter,
            args.initial_damping,
            relative_decrease_tol,
            args.reject,
            args.record_losses,
        )

    result: dict[str, Any] = {
        "bal_file": str(args.bal_file),
        "problem_name": dataset["problem_name"],
        "n_cameras": dataset["n_cameras"],
        "n_points": dataset["n_points"],
        "n_observations": dataset["n_observations"],
        "n_observations_raw": dataset["n_observations_raw"],
        "n_dropped_observations": dataset["n_dropped_observations"],
        "n_dropped_tracks": dataset["n_dropped_tracks"],
        "factor_filter": "match timeSFMBAL: keep only tracks with at least two measurements",
        "residual_convention": (
            "raw BAL/OpenGL projection; squared norm is equivalent to GTSAM's "
            "converted (u, -v) convention under unit noise"
        ),
        "torch_version": torch.__version__,
        "torch_cuda": torch.version.cuda,
        "environment": environment_info(device),
        "pypose_ba": pypose_result,
    }

    if args.gtsam_exe is not None:
        if not args.gtsam_exe.is_file():
            raise FileNotFoundError(args.gtsam_exe)
        result["gtsam"] = run_gtsam(args.gtsam_exe, args.bal_file)
        initial_errors = result["gtsam"]["initial_errors_from_stdout"]
        if initial_errors:
            python_error = pypose_result["initial_gtsam_style_error"]
            gtsam_error = initial_errors[0]
            abs_diff = abs(python_error - gtsam_error)
            result["initial_error_comparison"] = {
                "python_gtsam_style_error": python_error,
                "gtsam_first_initial_error_from_stdout": gtsam_error,
                "abs_diff": abs_diff,
                "relative_diff": abs_diff / max(1.0, abs(gtsam_error)),
            }
        final_results = result["gtsam"]["final_results_from_stdout"]
        if final_results:
            python_final = pypose_result["final_gtsam_style_error"]
            result["final_error_comparison"] = {
                "python_gtsam_style_error": python_final,
                "python_iterations_run": pypose_result["iterations_run"],
                "python_converged": pypose_result["converged"],
                "python_solve_loop_elapsed_s": pypose_result["solve_loop_elapsed_s"],
                "gtsam_solvers": [
                    {
                        **entry,
                        "abs_diff": abs(python_final - entry["final_error"]),
                        "relative_diff": abs(python_final - entry["final_error"])
                        / max(1.0, abs(entry["final_error"])),
                    }
                    for entry in final_results
                ],
            }

    text = json.dumps(result, indent=2)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text + "\n", encoding="utf-8")
    print(text)

    if args.print_gtsam_stdout and "gtsam" in result:
        print("\n--- GTSAM stdout ---")
        print(result["gtsam"]["stdout"])
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
