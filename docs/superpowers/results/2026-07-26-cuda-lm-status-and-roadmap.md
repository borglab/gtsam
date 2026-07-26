# CUDA Sparse LM: Status, Timing Breakdown, and Roadmap (2026-07-26)

Snapshot of the optimization campaign that started from the 2026-07-23
multidomain benchmark. All measurements: A100 80GB PCIe, CUDA 13.0,
32-core EPYC-Milan, Release build, worktree `direct-sparse-opt-in`,
branch `codex/direct-sparse-jacobian`.

## Current end-to-end results

Five-run medians, PCG backend at defaults (tol 1e-6), contemporaneous
CPU baselines, harness `timeCudaSparseLM` with `--objective-tol 0.02`
for PCG runs:

| Workload | CPU GTSAM | GPU at project start (cuDSS, 07-23) | GPU now (PCG) | vs CPU: start → now | GPU-vs-GPU gain |
| --- | ---: | ---: | ---: | --- | ---: |
| Dubrovnik-16 | 2.29 s | 0.677 s | **0.355 s** | 3.09x → **6.45x** | 1.90x |
| Dubrovnik-135 | 6.05 s | 3.186 s | **1.068 s** | 1.79x → **5.67x** | 2.98x |
| Pose2 w10000 | 1.52 s | 0.340 s | **0.305 s** | 4.22x → **4.99x** | 1.12x |
| Pose3 sphere | 3.68 s | 0.690 s | 0.712 s | 5.23x → 5.17x | ~1.0x |

Accuracy at these settings (relative final-objective difference vs the
CPU reference): bal16 ~3e-9, bal135 2.8e-4, pose2 2.8e-3, pose3 8e-6.
PCG at tolerance 1e-10 passes the strict 1e-8 parity, so the deltas are
LM-trajectory divergence from inexact steps, not solver error. The cuDSS
backend remains the default and passes 1e-8 parity everywhere.

## What was done (committed)

1. **`3d8536575` — matrix-free PCG backend** (`DevicePcgSolver.{h,cu}`,
   `params.linearSolver = Pcg`). Block-Jacobi preconditioner (warp-per-
   variable Gram build using the plan's column-pattern invariant,
   in-register Cholesky + explicit symmetric inverse per lambda), scaled
   warm start across lambda attempts, device-resident CG scalars with
   host checks every 10 iterations, true-residual verification,
   breakdown → finite delta (never throws). Eliminates cuDSS analysis
   (0.28–1.83 s) and SpGEMM H-pattern discovery entirely. Works in
   cuDSS-off builds. Default tol 1e-6 (sweep-validated; 1e-2 failed
   pose3 objective sanity, 1e-4→1e-6 costs ~nothing).
2. **`2d212390a` — parallel plan construction.** Three-phase rebuild of
   `SparseJacobianPlan`: parallel per-factor plans (TBB), serial prefix
   sums with the original overflow checks, parallel write-disjoint CSR
   fill. Plan 2.3–2.9x faster (bal135 314→108 ms); 1.11–1.13x
   end-to-end on BAL. Output byte-identical (fingerprints unchanged).
3. **`a3f066742` — SpMV preprocess.** Separate J/Jᵀ workspaces +
   `cusparseSpMV_preprocess` at setup. pcgSolve −7–17% everywhere
   (pose3 308→256 ms). Algorithm sweep: ALG_DEFAULT == ALG1 > ALG2;
   default kept. `GTSAM_PCG_SPMV_ALG=alg1|alg2` env knob remains.

## Negative results (implemented, measured, reverted — do not retry as-is)

Details in `2026-07-25-pcg-backend-benchmark.md`; raw JSONs under
`timing/cuda_sparse/results/2026-07-25-pcg/`.

- **cuDSS mtlayer threading (P1a)**: analysis is symbolic-factorization-
  dominated (bal135: 0.58 s reorder + 1.25 s symbolic); the threading
  layer parallelizes only reordering → ~0 gain.
  (`2026-07-25-cudss-mtlayer-reordering-benchmark.md`)
- **Eisenstat–Walker adaptive CG tolerance**: speedups were pure
  accuracy trade; the "loose early, tight late" regime never engages
  because gradients don't shrink enough before LM terminates.
- **CUDA-graph capture of the 10-iteration CG block**: solve time
  improved (pose3 −22%) but end-to-end regressed on 3/4 workloads via
  lost early-exit granularity + per-optimizer instantiation overhead.

## Current timing breakdown (PCG backend, 5-run medians, internal wall)

From `/tmp/final-pcg.json` (2026-07-26 run) plus the host-diagnostic
attribution pass (temporary instrumentation, since reverted):

| Cost cluster | bal16 (345 ms) | bal135 (1021 ms) | pose2 (291 ms) | pose3 (709 ms) |
| --- | ---: | ---: | ---: | ---: |
| linearize + CSR pack (CPU, TBB) | 63 · 18% | 153 · 15% | 52 · 18% | 87 · 12% |
| retract + trial error (CPU) | 41 · 12% | 136 · 13% | 52 · 18% | 97 · 14% |
| **Values-map churn (was unattributed)** | 83 · 24% | ~170 · 17% | 72 · 25% | 116 · 16% |
| hostZero + numericH2d | 17 · 5% | 49 · 5% | 20 · 7% | 109 · 15% |
| pcgSolve | 80 · 23% | 258 · 25% | 67 · 23% | 257 · 36% |
| one-time (plan + setup + init err) | 37 · 11% | 205 · 20% | 30 · 10% | 12 · 2% |
| small GPU stages | ~29 | ~34 | ~4 | ~19 |

### The Values-map churn finding (2026-07-26 diagnostic)

The former "unattributed host time" (18–23% of wall) is host-side
`Values` map handling in the attempt loop, itemized on bal135:
`currentValues = std::move(trialValues)` destroying the previous 90k-
entry map ≈ 94 ms; leftover `trialValues` destruction ≈ 47 ms;
`layout->toVectorValues(delta)` ≈ 30 ms; download/snapshot/finite-scan
≈ 6 ms. Confirmed clean: solver host wall ≈ pcgSolve (no hidden solver
overhead).

**Total trial-state handling (churn + retract + trial error): 30–45% of
wall on every workload — the single largest cost category.**

## Surviving vs dying costs (for the GPU migration)

Survives the planned device migration: pcgSolve (tuned), plan
(parallelized; cache-by-fingerprint later), setup (measured lean —
`deviceInitializeWall` is only 5–15 ms in PCG mode; the bulk of
persistentSetupWall is the pinned `HostSparseJacobian` alloc, which the
migration deletes). Dies with the migration: everything in the CPU
per-iteration cluster above (~55–65% of wall including churn).
Pre-migration optimization is **complete** — no remaining target of
meaningful size is migration-surviving.

## Next stage: device retract + trial error (slice 1 of the GPU migration)

Decision rationale: measured 30–45% of wall; smallest surface with the
biggest cut; builds the foundation (device values, manifold kernels,
per-type CPU fallback dispatch) that device linearization (slice 2)
requires; nothing thrown away.

Design requirements extracted from the measurements:
1. **Double-buffered device values** (current + trial) with O(1) swap on
   accept and no-op reject — a port that still materializes a host
   `Values` per attempt forfeits the largest measured term (the map
   churn). Host `Values` materialized once, at optimize() end.
2. Device retract kernels per manifold type: SE(3), SE(2), R^n cover all
   four benchmarks. Delta stays on device (kills per-attempt delta D2H +
   `toVectorValues`); finiteness checked device-side (PCG already has the
   flag pattern).
3. Device trial-error evaluation for resident factor types with
   per-factor CPU fallback (same `sendable`-style dispatch as the plan).
4. Slice 2 afterward: device linearization for the dominant factor types
   (BetweenFactor<Pose2/Pose3>, priors, projection), which also deletes
   hostZero + numericH2d + the pinned staging buffer. Longer term: AI-
   assisted generation of device factors for the long tail, gated by
   automated CPU/GPU parity + numeric-Jacobian oracle tests.

Projection if slice 1 delivers: bal135 ~1.07 → ~0.75 s (~8x vs CPU),
pose2 ~0.31 → ~0.21 s (~7x), pose3 ~0.71 → ~0.53 s (~7x).

## Methodology notes (learned the hard way)

- **Always A/B contemporaneously**: machine-state drift up to ~8% within
  a day produced a phantom regression (parallel-plan pose3) that a
  stash/rebuild interleaved A/B overturned. Never accept/reject against
  stored baselines.
- The benchmark's GPU objective check for inexact solvers uses
  `--objective-tol` (recorded in JSON as
  `gpu_objective_relative_tolerance`); CPU repeatability stays at 1e-8.
- bal135 is iteration-cap-bound in PCG (250/solve, 3 cap-hits at every
  tolerance tried): its 2.8e-4 objective gap is governed by
  `pcg.maxIterations`, not tolerance.
- PyPose/BAE comparison (2026-07-25): their PCG materializes JᵀJ, scalar
  Jacobi, no warm start, per-iteration syncs, a compounding-damping bug;
  their bal135 loop is faster (0.67 s) but lands 1.35% off the optimum
  vs our −0.03%. Their loop time argues for our slice-2 (their Jacobians
  come from batched GPU autodiff).

## Reproduce

```text
# current PCG state
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 1 --repeats 5 --datasets bal16,bal135,pose2,pose3 \
  --data-dir /home/ubuntu/cu-gtsam/examples/Data \
  --gpu-solver pcg --objective-tol 0.02 \
  --json out.json --csv out.csv
# cuDSS baseline: --gpu-solver cudss (no --objective-tol needed)
```

Related docs: `2026-07-23-cuda-sparse-lm-multidomain-benchmark.md`
(starting point), `2026-07-25-cudss-mtlayer-reordering-benchmark.md`,
`2026-07-25-pcg-backend-benchmark.md` (PCG design + negative results).
