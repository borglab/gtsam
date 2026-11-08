# Parked Dense Dual whitening experiment

This branch preserves the DAQP-style Dense Dual whitening implementation, tests,
selected benchmark report and phase/counter evidence. It is an experiment to
resume later; it is excluded from the production Dense QP PR.

## Checkpoint and resumption

- Branch: `codex/experiment-dense-dual-whitening`.
- Parent: `3ea2fe19f3`, containing optional Dense equality projection and cached
  native-Hessian Cholesky. The production branch remains at that parent.
- Resume with `git switch codex/experiment-dense-dual-whitening`, then
  `cmake -S . -B build` to refresh the source list.
- To transfer the experiment to a later base, cherry-pick this branch's final
  experiment commit. Resolve changes to Dense internals and revalidate original
  KKT checks before comparing timings.
- The detailed results are in [QpBenchmark.md](QpBenchmark.md), with the
  [interactive download](qp-benchmark-report.html). Charts require the internet.

## What the implementation contains

`internal/DenseDualQpSolver.cpp` implements working-set updates, immutable equality
rows with unrestricted multipliers, transformed-coordinate feasibility scans,
verified original-coordinate rays, deferred recovery and conditional refinement.
`DenseQpSolver` caches row-major normalized `A P R^-1` and a direct transformed
objective RHS. Curvature and transformed rows survive proximal center changes.
`DualQpSolver` dispatches Dense into this path and retains the old numerical path
when whitening is unavailable or unsuitable. Iteration limits retain their shared
budget. Sparse numerical code is unchanged.

## Why the experiment is parked

Whitening removes repeated original-variable solves, but eagerly transforming all
rows can cost more than the saved iterations. At explicit 1e-6 accuracy with
projection, WALK20 improves about 0.096 to 0.083 ms and BAL0 about 0.247 to 0.224 ms.
BAL10 regresses about 0.208 to 0.227 ms. Larger QUAD calls regress, especially when
projection is disabled. DAQP remains faster on the small cases. Its larger-case
results fail the original-input gap checks with the unchanged DAQP settings;
those failures remain visible in the report.

The paired campaign has 1,512 passing GTSAM calls and 252 DAQP calls, of which
144 pass the stricter original-input checks. All 756 diagnostic calls pass.
All 252 instrumented whitening calls use zero full primal solves and zero inverse
responses, with one cached workspace curvature factorization and one row
transformation per optimization. Construction/packing may do additional objective
validation/root work, which is included in the timing. Nine native suites pass;
Python has 582 passed, 6 skipped and 89 passed subtests. The documentation build
and interactive chart checks pass.

The main remaining costs are solver construction/packing, constraint
transformation, equality setup and original-QP certification. Those costs are more
promising investigation targets than adding more working-set machinery to this
version. Future repeated-MPC workloads may offer more opportunity to amortize
transformation, but the current caches intentionally last for one optimization.

## Local benchmark archive

Raw measurements and benchmark drivers remain untracked, following the existing
benchmark convention. A verified archive is stored outside the disposable Codex
worktree, in the main checkout's ignored `mpc_benchmark_local/saved-experiments/`
directory:

- File: `dense-dual-whitening-2026-11-07-3ea2fe19f.tar.gz`
- SHA-256: `5ba4b948a0509c6d8c46e834a348ca1ade40900ff80dc84744df42ef2622f8c7`

The archive includes an additional `RESUME.md`, all drivers and diagnostic adapter
sources, raw measurements/logs, baseline and tested shared libraries, the seven
public MPC fixtures and matching DAQP source. Paths in the private drivers must be
adjusted when restoring elsewhere. No raw G1/private capture is included. The
archive is local; the source, tests, aggregate results and this note are also
preserved by the remote experiment branch.
