# Shared CUDA linear solvers

The CUDA optimization code is organized around a **prepared system** boundary.
An optimizer frontend owns nonlinear semantics and produces one of three linear
system forms; `gtsam/linear/cuda` owns the reusable numerical solver session.
This keeps factor/projection code out of DenseCholesky, Cudss, and Pcg while
letting the general optimizer and SFM use the same solver lifecycle and stats.

## Ownership

- General CUDA LM linearizes arbitrary supported GTSAM factors into a sparse
  Jacobian. It owns factor evaluation, damping policy, retraction, nonlinear
  error, and model acceptance. Both its cuDSS normal-equation system and its
  matrix-free `J^T J + lambda D` operator dispatch through
  `LinearSolverSession`.
- SFM CUDA LM owns projection linearization, Schur/full-normal construction,
  point recovery, retraction, and LM acceptance. `SfmSchurProblem` builds
  and retains undamped camera `U`, point `V`, camera-point `W`, and gradient
  blocks once per outer iteration; dense, sparse, implicit, and recovery paths
  rebuild only their lambda-dependent state on retries.
- The shared layer owns dense cuSOLVER Cholesky, sparse cuDSS analysis/factor/
  solve, generic PCG recurrence, explicit sparse-SPD matvec/Jacobi support,
  backend validation, GTSAM Ordering expansion, and `LinearSolveStats`.

## SFM capability matrix

| Formulation | DenseCholesky | Cudss | Pcg |
|---|---:|---:|---:|
| Schur | yes | yes | yes |
| full normal | no | yes | yes |

Schur + DenseCholesky materializes a column-major reduced camera matrix from
the persistent blocks. Schur + Cudss scatters those blocks directly into a
stable camera-only upper CSR pattern derived from track co-visibility; it does
not create the dense matrix. Schur + Pcg applies
`U p - W V^-1 W^T p` directly from the same blocks and uses a 9-by-9
camera-block preconditioner. This is the scalable iterative SFM path.

Full-normal + Cudss materializes the undamped camera-plus-point upper CSR
system once per outer linearization, captures its diagonal, and restores that
diagonal before applying each lambda; it does not reaccumulate `J^T J` during
lambda retries.
Full-normal + Pcg applies `J^T(Jp) + lambda Dp` directly from the persistent
projection Jacobians with independent 9-by-9 camera and 3-by-3 point block-
Jacobi factors; it does not assemble CSR storage. It is primarily a
formulation/control baseline. Schur + Pcg still avoids the point variables and
is expected to be faster for BA.

The default remains Schur + DenseCholesky for compatibility. The specialized
parameter type inherits `LevenbergMarquardtParams`; therefore iteration,
lambda, convergence, damping, verbosity, and optional ordering have the same
source of truth as ordinary GTSAM LM. New code sets only the CUDA-specific
axes, `formulation` and `linear.backend`, independently. Legacy spellings such
as `dense-schur` and `cudss-full-normal` map onto those two axes without storing
a second combined solver enum.

The general CUDA LM parameters expose the same `linear` and `pcg` objects.
Only its preconditioner choice remains frontend-specific because it describes
how the general Jacobian producer builds block-Jacobi data, not how the shared
PCG recurrence runs. The old general-only backend enum is no longer a second
selection axis.

## GTSAM ordering and cuDSS

An `Ordering` contains variable keys, while cuDSS expects scalar indices.
`compileScalarPermutation` validates that every variable appears exactly
once and expands each key using its block dimension. A camera key expands to
nine consecutive scalars and an SFM point to three. Schur ordering contains
only cameras; full-normal ordering contains cameras and points.

The exact scalar vector is passed to cuDSS with `CUDSS_DATA_USER_PERM` before
analysis. The backend requests no additional reordering, reads the permutation
back after analysis, and checks that cuDSS retained it. RHS and solution
permutation remain cuDSS responsibilities. Ordering is rejected for dense and
PCG backends rather than silently ignored.

## Solver lifecycle and statistics

A solver session is created once per optimizer run. Stable sparse patterns and
ordering are analyzed once, while numerical factorization/solve occurs once per
lambda attempt. Dense analysis caches the dimension. PCG allocates recurrence
storage once and reports convergence, accumulated iterations, and maximum-
iteration hits. `LinearSolveStats` records backend, ordering application,
analysis/factorization/solve counts, aggregate PCG cap/breakdown counts,
convergence-check D2H bytes/time, and backend timings. Frontend assembly,
transfers, model evaluation, and retraction stay in optimizer-specific profiles
and are not counted as backend solve time.

## Build and verification

```bash
cmake -S . -B build-cuda -DGTSAM_ENABLE_CUDA=ON -DGTSAM_ENABLE_CUDSS=ON
cmake --build build-cuda -j2 --target testCudaLinearSolver \
  testCudaSparseJacobian testCudaSparseLevenbergMarquardt testCudaSfm \
  timeCudaSparseLM timeCudaSFMBAL
ctest --test-dir build-cuda --output-on-failure \
  -R 'testCudaLinearSolver|testCudaSparseJacobian|testCudaSparseLevenbergMarquardt|testCudaSfm'
```

Focused memory validation:

```bash
compute-sanitizer --tool memcheck --error-exitcode=99 \
  ./build-cuda/gtsam/linear/tests/testCudaLinearSolver
compute-sanitizer --tool memcheck --error-exitcode=99 \
  ./build-cuda/gtsam/slam/tests/testCudaSfm
```

BAL examples:

```bash
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --cuda-lm \
  --cuda-linear-solver dense-schur problem.txt
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --cuda-lm \
  --cuda-linear-solver cudss-schur problem.txt
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --cuda-lm \
  --cuda-linear-solver pcg-schur problem.txt
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --cuda-lm \
  --cuda-linear-solver cudss-full-normal problem.txt
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --cuda-lm \
  --cuda-linear-solver pcg-full-normal problem.txt
./build-cuda/timing/cuda_sparse/timeCudaSparseLM --help
```

The matrix benchmark has stable configuration names and JSON/CSV records:

```bash
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --list-configurations
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --cuda-lm \
  --configuration schur-cudss-gtsam --output-format json problem.txt
timing/sfm_ba/run_cuda_sfm_bal_benchmarks.sh problem.txt results build-cuda/timing/sfm_ba/timeCudaSFMBAL
```

Each machine-readable row records formulation, backend, actual representation,
ordering application, system dimension/nonzeros,
analysis/factorization/solve counts, PCG convergence,
objectives, transfer bytes, frontend wall time, backend time, accepted steps,
and lambda attempts. The matrix runner gates every row against the dense-Schur
objective, requires every PCG solve to avoid the iteration cap and breakdown,
requires the last solve to converge, and rejects representation/order metadata
that does not match the requested configuration. PCG convergence polling is
included in reported D2H totals. The GTSAM-ordering rows
compute COLAMD at key/block level, expand it to scalar indices through
`compileScalarPermutation`, and verify that cuDSS retained the supplied
permutation.

The named PCG benchmark rows use a correctness-oriented iterative profile
(`relativeTolerance=1e-6` and at most 5000 iterations; the general benchmark
requires 0.1% final-objective agreement). The runners reject any row in which
an inner solve reaches that cap, breaks down, or the last solve does not
converge. These are benchmark controls, not changes to the public
`PcgOptions` defaults. An inexact PCG solve can legitimately take a
different LM trajectory, so comparing it with the direct row at the direct
solver's `1e-8` objective tolerance is not a meaningful pass/fail gate.

In CUDA builds without cuDSS, the same SFM benchmark executable still exposes
Schur+dense, Schur+PCG, and full-normal+PCG. Only the cuDSS rows are rejected.
