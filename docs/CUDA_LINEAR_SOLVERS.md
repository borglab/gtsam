# Shared CUDA linear solvers

The CUDA optimization code is organized around a **prepared system** boundary.
An optimizer frontend owns nonlinear semantics and produces one of three linear
system forms; `gtsam/linear/cuda` owns the reusable numerical solver session.
This keeps factor/projection code out of DenseCholesky, Cudss, and Pcg while
letting the general optimizer and SFM use the same solver lifecycle and stats.

## Ownership

- General CUDA LM linearizes arbitrary supported GTSAM factors into a sparse
  Jacobian. It owns factor evaluation, damping policy, retraction, nonlinear
  error, and model acceptance. Its cuDSS direct path dispatches the prepared
  normal equations through `CudaLinearSolverSession`; its established
  matrix-free PCG kernels remain optimized for the sparse-Jacobian layout.
- SFM CUDA LM owns projection linearization, Schur/full-normal construction,
  point recovery, retraction, and LM acceptance. `CudaSfmSchurProblem` keeps
  projection Jacobians alive once per outer iteration and rebuilds only the
  lambda-dependent system on retries.
- The shared layer owns dense cuSOLVER Cholesky, sparse cuDSS analysis/factor/
  solve, generic PCG recurrence, explicit sparse-SPD matvec/Jacobi support,
  backend validation, GTSAM Ordering expansion, and `CudaLinearSolveStats`.

## SFM capability matrix

| Formulation | DenseCholesky | Cudss | Pcg |
|---|---:|---:|---:|
| Schur | yes | yes | yes |
| full normal | no | yes | yes |

Schur + DenseCholesky materializes a column-major reduced camera matrix. Schur
+ Cudss scatters directly into a stable camera-only upper CSR pattern derived
from track co-visibility; it does not create the dense matrix. Schur + Pcg
applies `U p - W V^-1 W^T p` implicitly and uses a 9-by-9 camera-block
preconditioner. This is the scalable iterative SFM path.

Full-normal + Cudss materializes the camera-plus-point upper CSR system.
Full-normal + Pcg currently applies that explicit sparse SPD system with a
Jacobi preconditioner. It is primarily a formulation/control baseline; Schur
+ Pcg avoids the larger system and is expected to be faster for BA.

The default remains Schur + DenseCholesky for compatibility. New code should
set `CudaSfmLevenbergMarquardtParams::formulation` and `linear.backend`
independently. Legacy spellings such as `dense-schur` and
`cudss-full-normal` map onto those two axes.

## GTSAM ordering and cuDSS

An `Ordering` contains variable keys, while cuDSS expects scalar indices.
`CompileCudaScalarPermutation` validates that every variable appears exactly
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
iteration hits. `CudaLinearSolveStats` records backend, ordering application,
analysis/factorization/solve counts, and backend timings. Frontend assembly,
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
