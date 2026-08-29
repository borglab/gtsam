# General CUDA LM Python Binding Design

## Goal

Expose the practical, user-facing portion of the general CUDA
Levenberg-Marquardt optimizer through the existing `gtsam.cuda` Python
namespace when GTSAM is built with CUDA support.

## Scope

The binding will expose enough of the shared CUDA linear-solver configuration,
general CUDA LM parameters, optimizer, and result to configure and run an
optimization and understand its outcome. Detailed profiling structures remain
out of scope: stage timings, transfer counts, sparse-system dimensions,
per-attempt traces, scalar permutations, and the complete linear-solver
statistics will stay C++-only.

## Wrapper Structure

CUDA wrapper declarations will follow the C++ ownership boundaries:

- `gtsam/linear/cuda/cuda_linear.i` will declare the shared
  `LinearSolverType`, `LinearSolverOptions`, and `PcgOptions` types.
- `gtsam/nonlinear/cuda/cuda_nonlinear.i` will declare the general CUDA LM
  parameter, status, result, and optimizer types.
- `gtsam/sfm/cuda/cuda_sfm.i` will continue to declare the CUDA SFM API but
  will use the shared `LinearSolverType` declaration rather than redeclaring
  the enum.
- `python/CMakeLists.txt` will append the three CUDA interface files in shared,
  general, then SFM order under `GTSAM_ENABLE_CUDA`.

All declarations will land in the existing `gtsam::cuda` namespace and will
therefore appear under `gtsam.cuda` in Python.

## Python API

The shared configuration surface will include:

- `LinearSolverType` with `DenseCholesky`, `Cudss`, and `Pcg` values.
- `LinearSolverOptions()` with writable `backend`.
- `PcgOptions()` with writable `maxIterations`, `relativeTolerance`,
  `warmStart`, and `convergenceCheckInterval`.

The general optimizer surface will include:

- `SparseLevenbergMarquardtParams()`, derived from
  `LevenbergMarquardtParams`, with writable `fallbackOnUnsupported`,
  `collectTiming`, `collectAttemptTrace`,
  `validateStructureEveryIteration`, `linear`, and `pcg` fields.
- `SparseLevenbergMarquardtBackend`,
  `SparseLevenbergMarquardtTerminationReason`,
  `SparseLevenbergMarquardtFallbackReason`, and `DirectJacobianFailure` enums.
- `DirectJacobianStatus()` with `failure`, `factorIndex`, `detail`, and `ok()`.
- `SparseLevenbergMarquardtResult()` with backend/fallback/termination state,
  fallback status and detail, iteration and solve counts, and initial/final
  error and lambda values. Profiling-heavy nested fields are omitted.
- `SparseLevenbergMarquardtOptimizer(graph, initialValues, params)` with
  `optimize()`, `values()`, `error()`, `params()`, and `result()`.

The `.i` declarations will match the C++ signatures and parameter names
exactly. No new C++ production API is required.

## Behavior and Errors

Construction and optimization will preserve the existing C++ validation and
exception behavior. Unsupported backend choices, unavailable CUDA runtimes,
and incompatible graph structures will continue to surface as translated
Python exceptions or as the existing CPU-fallback result when fallback is
enabled. The wrapper will not add a second validation layer.

The general optimizer does not support `DenseCholesky`; the enum remains
available because it is shared with CUDA SFM, while the general optimizer's C++
validation remains responsible for rejecting it.

## Testing

A new CUDA-conditional Python test module will verify:

1. The shared solver and PCG options can be created and assigned through the
   general LM parameters.
2. The practical result/status types and enum values are present and writable
   where the C++ API exposes public fields.
3. A small nonlinear factor graph can construct and run the general optimizer
   through Python using PCG, with runtime-unavailable errors skipped in the
   same manner as the CUDA SFM tests.
4. The existing CUDA SFM wrapper still uses the same shared
   `LinearSolverType` and remains importable.

The test file will only be copied and selected when `GTSAM_ENABLE_CUDA` is on,
matching `test_CudaSfm.py`. Validation will include wrapper-generator tests, a
CUDA-enabled Python build, the focused new Python test, the existing CUDA SFM
Python test, and `git diff --check`.

## Compatibility

Non-CUDA builds are unchanged because all new interface and test wiring is
inside `GTSAM_ENABLE_CUDA`. Existing Python names under `gtsam.cuda` remain
unchanged. Moving the shared enum declaration between interface files changes
only wrapper organization, not its Python-qualified name or C++ definition.
