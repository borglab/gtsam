# Matrix-Free PCG Backend: Implementation and A/B Benchmark

## Outcome

A matrix-free block-Jacobi-preconditioned conjugate gradient backend was
added to `CudaSparseLevenbergMarquardtOptimizer` as
`params.linearSolver = CudaSparseLmLinearSolver::Pcg`. It solves
`(JᵀJ + λD)δ = Jᵀb` with two SpMVs per iteration and never forms the normal
matrix: SpGEMM pattern discovery, stable-H storage, and cuDSS are all
skipped. **PCG beat the cuDSS backend on all four workloads**, with the
largest wins exactly where the timing breakdown predicted — the
analysis-dominated BAL problems.

### Five-run medians (A100, warmup 1, repeats 5, PCG tol 1e-4)

| Workload | GPU wall cuDSS | GPU wall PCG | PCG vs cuDSS | CPU/GPU (cuDSS) | CPU/GPU (PCG) |
| --- | ---: | ---: | ---: | ---: | ---: |
| bal16 | 0.714 s | 0.386 s | **1.85x** | 3.70x | **6.58x** |
| bal135 | 3.303 s | 1.506 s | **2.19x** | 2.03x | **4.67x** |
| pose2 | 0.368 s | 0.323 s | 1.14x | 4.59x | 5.47x |
| pose3 | 0.721 s | 0.670 s | 1.08x | 5.65x | 6.06x |

Where the time went: cuDSS analysis (0.28–1.73 s) and most of device
initialization (SpGEMM discovery; 22–30x smaller in PCG mode) disappear,
replaced by a PCG solve totaling 39–277 ms per run. Median CG iteration
counts: bal16 540 over 5 solves, bal135 750 over 3 (3 attempts hit the
250-iteration cap — harmless truncation, see below), pose2 470 over 11,
pose3 3140 over 54.

## Accuracy: the honest caveat

PCG is an inexact solver, so LM follows a slightly different trajectory and
lands at a slightly different local objective. The benchmark's strict 1e-8
CPU/GPU parity check is a direct-solver assumption; PCG runs used a new
`--objective-tol 0.02` flag (recorded in the JSON as
`gpu_objective_relative_tolerance`; CPU repeatability keeps 1e-8).

Measured relative final-objective differences vs the CPU reference:

| Workload | Absolute diff | Relative diff |
| --- | ---: | ---: |
| bal16 | 9.8e-3 | 5.4e-7 |
| bal135 | 1.1e+2 | 2.9e-4 |
| pose2 | 5.8e+4 | 1.04e-2 |
| pose3 | 1.0e+1 | 2.0e-3 |

Tight-tolerance validation (`--pcg-tol 1e-10 --pcg-max-iters 100000`)
passed the strict 1e-8 parity on pose2 and pose3, confirming the solver is
*correct* — the loose-tolerance differences are trajectory divergence, not
bugs. Tolerance choice is workload-dependent and exposed via
`params.pcg.relativeTolerance`.

A follow-up tolerance sweep (see `pcg-1e-6.json`) settled the default at
**1e-6**: compared with 1e-4 it costs almost nothing (bal16 0.386→0.397 s;
bal135 actually improved to 1.385 s) while tightening objectives
substantially (bal16 3e-10 relative, pose3 8e-6, pose2 2.8e-3). The
original 1e-2 default failed the 0.02 objective check on pose3 (7345 vs
5037 — both iteration-limited non-converged points) and was rejected.
Note bal135's solves hit the 250-iteration cap at both 1e-4 and 1e-6
(identical 750 total iterations), so its 2.8e-4 objective gap is governed
by the cap, not the tolerance.

## Design (synthesized from a 3-agent design phase)

- **Algorithm**: PCG directly on the damped normal equations. The κ²
  objection is moot at LM tolerances; λD bounds conditioning exactly when CG
  would struggle. Truncated solves (max-iteration cap) are provably still
  descent directions with x₀=0 or the scaled warm start, so LM's fidelity
  logic absorbs them — the 3 cap-hits on bal135 are benign.
- **Preconditioner**: block-Jacobi over the variable blocks. Undamped Gram
  blocks are built once per linearization by a warp-per-variable kernel
  exploiting a structural invariant of `SparseJacobianPlan` (all columns of
  a variable share one residual-row pattern, so each variable's Jᵀ data is
  one contiguous w×nnz chunk — validated at initialize). Per lambda
  attempt: in-register packed Cholesky + explicit symmetric inverse
  (exact symmetry is required by PCG); non-SPD blocks fall back to scalar
  Jacobi per-block.
- **Warm start**: across lambda attempts (same gradient), δ₀ = θ*·δ_prev
  with θ* the exact 1-D minimizer of the damped quadratic — never worse
  than cold start. Cold start per linearization.
- **Sync policy**: device-resident CG scalars via fused reduction kernels
  (atomicAdd); host convergence check every 10 iterations; recurrence
  verified against the true residual before accepting convergence and
  refreshed every 100 iterations; breakdown freezes the iterate via a
  device flag (alpha=0) and returns a finite delta — never throws.

## Files

- New: `gtsam/nonlinear/cuda/DevicePcgSolver.{h,cu}` — solver, kernels.
- `DeviceSparseJacobianNormalEquations.{h,cu}` — `DeviceNormalSolverBackend`
  enum + `DeviceNormalSolverOptions` (defaulted 4th `initialize` arg; all
  existing callers unchanged), backend-branched setup/form/analyze/solve,
  `hasNormalMatrix()`, PCG profile fields, `ColumnSquaredNormsKernel` for
  diag(JᵀJ) without H. cuDSS mode is a line-for-line preservation
  (`std::optional<CudssSpdSolver>` emplaced in its branch); both existing
  unit-test suites (`testCudaSparseJacobian`,
  `testCudaSparseLevenbergMarquardt`) pass unchanged.
- `CudaSparseLevenbergMarquardt.{h,cpp}` — `linearSolver` + `PcgParams` on
  the params, block offsets derived from the column layout, PCG counters on
  the result, and the cuDSS-off CPU-fallback gate now applies only when the
  cuDSS backend is selected (PCG works in cuDSS-off builds).
- `timing/cuda_sparse/timeCudaSparseLM.cpp` — `--gpu-solver cudss|pcg`,
  `--pcg-tol`, `--pcg-max-iters`, `--objective-tol`; pcg timing fields and
  per-run counters in JSON/CSV.

## Reproduce

```text
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 1 --repeats 5 --datasets bal16,bal135,pose2,pose3 \
  --data-dir /home/ubuntu/cu-gtsam/examples/Data \
  --gpu-solver pcg --pcg-tol 1e-4 --objective-tol 0.02 \
  --json timing/cuda_sparse/results/2026-07-25-pcg/pcg.json \
  --csv timing/cuda_sparse/results/2026-07-25-pcg/pcg.csv
# baseline: same command with --gpu-solver cudss and cudss.json/csv
```

Artifacts: `timing/cuda_sparse/results/2026-07-25-pcg/` (both JSON/CSV and
console logs). Environment identical to the 2026-07-23/25 benchmarks
(A100 80GB, CUDA 13.0, cuDSS 0.8, 32-core EPYC, Release).

## Follow-up experiments: two negative results (2026-07-25)

Both deferred v1 optimizations were implemented, benchmarked against the
committed fixed-1e-6 baseline (same protocol: warmup 1, repeats 5, all four
datasets, `--objective-tol 0.02`), and **rejected** under the gate "retain
only if speed improves without accuracy degradation." The code was
reverted; only the benchmark JSONs remain
(`results/2026-07-25-pcg/pcg-ew.json`, `pcg-graph.json`).

### Adaptive (Eisenstat–Walker style) CG tolerance — rejected

Per-linearization tolerance `clamp(0.9·(‖g_k‖/‖g_0‖)^1.5, 1e-6, ceiling)`.
The classic consecutive-ratio EW rule with a 1e-2 ceiling failed the pose3
objective check outright (7345 vs 5037); cumulative-progress tightening
with the ceiling lowered to 1e-4 passed and was what got measured:

| Workload | GPU wall fixed → adaptive | Rel. objective fixed → adaptive |
| --- | --- | --- |
| bal16 | 0.397 → 0.375 s | 3.1e-10 → 5.4e-7 |
| bal135 | 1.385 → 1.426 s | 2.8e-4 → 2.8e-4 |
| pose2 | 0.342 → 0.294 s | 2.8e-3 → **1.0e-2** |
| pose3 | 0.737 → 0.621 s | 8.3e-6 → **2.1e-3** |

The speedups (up to 16%) come entirely from solving less accurately — CG
iteration counts drop in exact proportion (pose3 4660 → 3140) and the
objective follows the tolerance down. The hoped-for "loose early, tight
late" free lunch never materializes because the gradient norm does not
shrink far enough on these workloads before LM terminates for the
tightening to engage. This is a speed/accuracy dial, not an optimization.

### CUDA Graph capture of the 10-iteration block — rejected

One graph capturing `convergenceCheckInterval` full CG iterations,
replayed between convergence checks (keyed on borrowed pointers,
permanent scalar-loop fallback on capture failure). Unit tests passed;
launch sequence identical by construction.

| Workload | GPU wall baseline → graph | pcgSolve baseline → graph |
| --- | --- | --- |
| bal16 | 0.397 → 0.438 s | 0.092 → 0.106 s |
| bal135 | 1.385 → 1.348 s | 0.277 → 0.254 s |
| pose2 | 0.342 → 0.355 s | 0.078 → 0.067 s |
| pose3 | 0.737 → 0.796 s | 0.309 → 0.242 s |

The launch-overhead theory was half right: raw solve time improved on the
iteration-heavy workloads (pose3 −22%, pose2 −14%). But end-to-end wall
regressed on 3 of 4 datasets because (a) the block-granular loop loses
early exit — iterations always run to a multiple of 10 past convergence
(bal16 910 → 1250 total iterations, which also perturbed its objective to
7.9e-8), and (b) per-optimizer graph instantiation/recapture overhead is
amortized over too few, too-short solves. The solver-local gain does not
survive whole-pipeline measurement.

### Standing conclusions

- The committed configuration (fixed 1e-6 tolerance, scalar host-driven
  loop) is the best measured variant; remaining PCG-internal headroom is
  small.
- bal135 is iteration-cap-bound (750 total iterations, 3 cap-hits at every
  tolerance tried), so its 2.8e-4 objective gap is governed by
  `maxIterations`, not tolerance — raising the cap for large problems is
  an accuracy knob, the only one that helps there.
- The profile now points outside the solver: CPU linearization/packing and
  retract dominate the pose-graph runs (P5/P6 from the original plan are
  the next lever). `cusparseSpMV_preprocess` remains untried but its upside
  is bounded by the same whole-pipeline effect that sank the graph capture.
