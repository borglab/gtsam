# CUDA SFM Levenberg-Marquardt Implementation Notes

This document explains the current CUDA SFM / bundle-adjustment Levenberg-Marquardt path and the user-facing benchmark modes added around it. The implementation has two related goals:

- expose a CPU-LM-compatible public CUDA BA path for supported `GeneralSFMFactor<SfmCamera, Point3>` graphs, so callers can use a familiar `NonlinearOptimizer` entry point and `LevenbergMarquardtParams`;
- keep the lower-level BAL/SFM CUDA path easy to benchmark, including solver selection, warmup, and timing breakdowns from `timeSFMBAL`.

The CUDA path is still specialized. It is closer to CPU LM behavior than the first CUDA backend, but it is not a general nonlinear optimizer and should not be described as numerically identical to CPU LM.

## Public API Shape

The public CUDA SFM LM declarations live in `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h`.

The low-level API is still `SfmData` based:

```cpp
using namespace gtsam::cuda;

CudaSfmLevenbergMarquardtParams params;
params.maxIterations = 20;
params.relativeErrorTol = 0.01;
params.linearSolver = CudaSfmLinearSolverType::DenseSchur;

CudaSfmLevenbergMarquardtResult result = OptimizeCudaSfm(data, params);
```

The convenience overload assigns default keys `C(i)` for cameras and `P(j)` for points. A second overload accepts explicit camera and point key vectors when the caller already has graph keys to preserve:

```cpp
CudaSfmLevenbergMarquardtResult result =
    OptimizeCudaSfm(data, cameraKeys, pointKeys, params);
```

The new graph-facing API is `CudaSfmLevenbergMarquardtOptimizer`, a `NonlinearOptimizer` subclass. It takes a `NonlinearFactorGraph`, initial `Values`, regular `LevenbergMarquardtParams`, and optionally a CUDA linear solver selection:

```cpp
using namespace gtsam;
using namespace gtsam::cuda;

LevenbergMarquardtParams params = LevenbergMarquardtParams::CeresDefaults();
params.maxIterations = 20;
params.relativeErrorTol = 0.01;

CudaSfmLevenbergMarquardtOptimizer lm(
    graph, initial, params, CudaSfmLinearSolverType::DenseSchur);

const Values& optimized = lm.optimize();
const CudaSfmLevenbergMarquardtResult& backend = lm.result();
```

Important API details:

- `CudaSfmLinearSolverType` currently supports `DenseSchur` and `CudssFullNormal` (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h:12`).
- `CudaSfmLevenbergMarquardtParams` mirrors the LM fields needed by the backend, including lambda bounds, error tolerances, model fidelity, fixed/non-fixed lambda behavior, and diagonal damping controls (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h:17`).
- `CudaSfmLevenbergMarquardtResult` reports objective values, setup/solve/download timings, accepted outer iterations, inner lambda attempts, accepted steps, final lambda, and optional downloaded `Values` (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h:36`).
- `CudaSfmLevenbergMarquardtOptimizer::optimize()` is implemented; `iterate()` intentionally throws and tells callers to use `optimize()` (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:271`).
- `OptimizeCudaSfm` requires `GTSAM_ENABLE_CUDSS=ON`; without it the function throws (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:287`).

## Graph And Values Conversion

The graph API converts a supported `NonlinearFactorGraph` plus initial `Values` into `SfmData` with sidecar key vectors using `ConvertGeneralSfmGraphToCudaSfmData` (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:179`).

Conversion behavior:

- all `SfmCamera` entries in `initialValues` become CUDA camera slots and are recorded in `cameraKeys`;
- all `Point3` entries become SFM tracks and are recorded in `pointKeys`;
- each graph factor must dynamically cast to `GeneralSFMFactor<SfmCamera, Point3>`;
- null factor slots are skipped;
- the factor noise model must be unit noise, or absent/unit according to `noiseModel()->isUnit()`;
- factor `key1()` must map to a converted camera key and `key2()` must map to a converted point key;
- each supported factor appends one measurement to the corresponding SFM track using the converted camera slot and the factor measurement.

The explicit key vectors matter because `PackSfmValues` now accepts caller-provided camera and point keys and stores those keys in the `DeviceValues` blocks (`gtsam/slam/cuda/CudaSfmValues.h:46`). `DownloadSfmValues` later reconstructs host `SfmCamera` and `Point3` values using the same block keys (`gtsam/slam/cuda/CudaSfmValues.h:112`).

The graph wrapper preserves unrelated values by merging CUDA results into the original `Values`: it updates only the downloaded optimized SFM keys and leaves other initial keys untouched (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:254`). The test `OptimizesGeneralSfmGraphWithArbitraryKeys` covers arbitrary camera/point symbols and an unrelated `Point2` value that survives unchanged (`gtsam/slam/tests/testCudaSfm.cpp:774`).

Current conversion limitations:

- only `GeneralSFMFactor<SfmCamera, Point3>` is accepted;
- non-unit Gaussian noise, robust noise, priors, smart factors, between factors, and mixed factor graphs are rejected;
- camera values must be `SfmCamera`, which is `PinholeCamera<Cal3Bundler>`, with a 9-dimensional tangent block;
- landmark values must be `Point3`;
- separate calibration keys are not supported;
- fixed variables are not modeled as fixed in this conversion. Every converted camera and point value participates in the CUDA state.

These restrictions are deliberate: the CUDA kernels operate on a BAL-style packed camera block, point block, and projection observation batch.

## LM Loop Behavior

The graph optimizer translates `LevenbergMarquardtParams` into backend params in `ConvertLmParams` (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:121`). The mapping includes:

- `maxIterations`;
- `lambdaInitial`;
- `lambdaFactor` as the lambda-up factor;
- reciprocal `lambdaFactor` as the fixed lambda-down factor;
- lambda upper/lower bounds;
- relative, absolute, and absolute-error stopping tolerances;
- `minModelFidelity`;
- `useFixedLambdaFactor`;
- diagonal damping flags and clamp bounds from `dampingParams`;
- `downloadOptimizedValues = true` for the graph-facing optimizer.

The backend starts by packing values, building the projection batch, and computing the initial objective (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:303`). It returns early for zero requested iterations, zero dimension, or an initial error already below `errorTol` (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:340`).

The solve loop now has an outer accepted-step loop and an inner lambda-search loop (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:380`). `result.iterations` counts accepted nonlinear iterations. `result.innerIterations` counts linear solve attempts, including rejected lambda trials.

Each inner attempt:

1. optionally computes a current Hessian diagonal for diagonal damping;
2. solves either the dense Schur system or the full sparse normal system at the current lambda;
3. computes the predicted linearized cost change with `ComputeCudaSfmLinearizedErrorChange`;
4. applies the delta to a trial state only when the predicted change is nonnegative;
5. computes the actual nonlinear cost change;
6. computes model fidelity as `actualCostChange / linearizedCostChange`;
7. accepts the step only when model fidelity is above `minModelFidelity`.

This is the main CPU-LM compatibility improvement. The previous simple behavior accepted any lower nonlinear error and otherwise multiplied lambda. The new path compares actual improvement to the linear model prediction, retries lambda on failed trials, and stops through the same family of LM tolerances.

Lambda behavior:

- With `useFixedLambdaFactor = true`, rejected trials multiply lambda by `lambdaUpFactor`, and accepted trials multiply lambda by `lambdaDownFactor`.
- With `useFixedLambdaFactor = false`, rejected trials multiply by an adaptive `currentFactor`, and accepted trials use the cubic model-fidelity update `max(1/3, 1 - pow(2 * modelFidelity - 1, 3))`.
- Lambda is clamped below by `lambdaLowerBound` after successful decreases, and the search terminates when lambda reaches `lambdaUpperBound`.

Stopping behavior:

- accepted steps call `CheckCudaLmConvergence`, which checks `errorTol`, `relativeErrorTol`, and `absoluteErrorTol` (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:164`);
- failed lambda search stops when lambda reaches the upper bound;
- very small actual cost change, measured against `relativeErrorTol * currentError`, stops the lambda search (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:444`);
- non-finite current error exits the outer solve loop.

The result records `finalError`, `finalLambda`, accepted steps, inner attempts, and timing metadata at the end of the synchronized solve loop (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:473`).

## Linear Solver And Damping Plumbing

There are two solver paths behind the same LM loop.

### Dense Schur

`CudaSfmDenseSchurSolver` eliminates points and solves a dense camera system. The new overload accepts a precomputed damping diagonal (`gtsam/slam/cuda/CudaSfmDenseSchurSolver.h:23`).

When diagonal damping is enabled, the point normal blocks, camera diagonal entries, and point-delta recovery all use `lambda * dampingDiagonal[i]` instead of plain `lambda` (`gtsam/slam/cuda/CudaSfmDenseSchurSolver.cu:240`, `gtsam/slam/cuda/CudaSfmDenseSchurSolver.cu:451`, `gtsam/slam/cuda/CudaSfmDenseSchurSolver.cu:464`). Without a damping diagonal, the path keeps the old identity damping behavior.

Input checking verifies that the damping diagonal length matches `9 * numCameras + 3 * numPoints` (`gtsam/slam/cuda/CudaSfmDenseSchurSolver.cu:536`).

### Full Normal Equations

The full-normal path accumulates a sparse normal equation system and solves it with cuDSS. `DeviceSparseNormalEquations` now has a second damping overload:

```cpp
void addDiagonalDamping(double lambda,
                        const CudaDeviceArray<double>& diagonal,
                        cudaStream_t stream = nullptr);
```

The plain overload adds `lambda` to each diagonal. The scaled overload validates the supplied diagonal size, checks that all matrix diagonal entries exist, and adds `lambda * diagonal[row]` on device (`gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h:78`, `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.cu:105`).

### Hessian Diagonal Helper

`ComputeCudaSfmHessianDiagonal` computes the diagonal of `J^T J` for the current projection batch (`gtsam/slam/cuda/CudaSfmProjectionLinearization.h:27`). It launches over observations, atomically accumulates squared camera and point Jacobian columns into a vector with layout:

```text
[9 values per camera][3 values per point]
```

It then clamps every entry to `[minDiagonal, maxDiagonal]` (`gtsam/slam/cuda/CudaSfmProjectionLinearization.cu:335`). This diagonal feeds both dense Schur and full-normal damping when `params.diagonalDamping` is true.

### Linearized Error-Change Helper

`ComputeCudaSfmLinearizedErrorChange` computes the predicted decrease used for model fidelity (`gtsam/slam/cuda/CudaSfmProjectionLinearization.h:32`). It materializes a projection linearization, evaluates:

```text
oldLinearizedError = 0.5 * ||r||^2
newLinearizedError = 0.5 * ||r + J * delta||^2
return oldLinearizedError - newLinearizedError
```

The implementation reduces block sums on device and then downloads those block sums for the final host reduction (`gtsam/slam/cuda/CudaSfmProjectionLinearization.cu:403`). That makes model-fidelity behavior available now, with a performance cost that can be optimized later.

## Values Writeback

The CUDA value packer stores cameras and points as two `DeviceValues` blocks with tangent dimensions 9 and 3. For low-level `OptimizeCudaSfm(data, params)`, the default key convention is still `C(i)` and `P(j)` (`gtsam/slam/cuda/CudaSfmValues.h:80`).

For graph API calls, conversion passes the original camera and point keys to `PackSfmValues`, so `DownloadSfmValues` returns optimized values under the caller's symbols. `CudaSfmLevenbergMarquardtOptimizer::optimize()` merges those values into a copy of the original `Values`, recomputes `graph().error(merged)`, and installs a new `NonlinearOptimizerState` with the optimized values and accepted iteration count (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:260`).

The low-level params still support `downloadOptimizedValues = false`, which is useful for benchmark runs that only need objective/timing metadata. The test `CanSkipOptimizedValueDownload` covers this behavior (`gtsam/slam/tests/testCudaSfm.cpp:847`).

## Benchmark Modes

`timing/timeSFMBAL.cpp` now exposes two CUDA LM modes in addition to the existing CPU timings and structure-only CUDA mode:

```text
Usage: timeSFMBAL [--colamd] [--profile] [--cuda-structure-only]
                  [--cuda-lm] [--cuda-lm-graph]
                  [--cuda-linear-solver dense-schur|cudss-full-normal]
                  [--cuda-warmup-file FILE]
                  [--benchmark-action-json FILE] [BALfile ...]
```

The relevant option parsing is in `timing/timeSFMBAL.cpp:230`.

### `--cuda-lm`

`--cuda-lm` runs the low-level `SfmData` backend directly (`timing/timeSFMBAL.cpp:432`). It:

- reads the BAL file into `SfmData` before timing;
- sets backend params to `maxIterations = 20`, `relativeErrorTol = 0.01`, and `downloadOptimizedValues = false`;
- selects `DenseSchur` by default, or `CudssFullNormal` when `--cuda-linear-solver cudss-full-normal` is passed;
- measures wall time around `OptimizeCudaSfm(db, params)`;
- prints wall time, selected solver, backend solve-loop time, backend measured total, setup breakdown, initial/final error, accepted iterations, and accepted steps.

Example command structure:

```bash
./build-cuda-cudss-on/timing/timeSFMBAL \
  --cuda-lm \
  --cuda-linear-solver dense-schur \
  --cuda-warmup-file examples/Data/dubrovnik-16-22106-pre.txt \
  examples/Data/dubrovnik-88-64298-pre.txt
```

The current code prints output shaped like:

```text
CUDA LM: <seconds> s
CUDA LM linear solver: dense-schur
CUDA LM solve loop: <seconds> s
CUDA LM measured total: <seconds> s
...
Initial error: <value>
Final error: <value>, iterations: <accepted>, accepted: <accepted>
```

No new benchmark result numbers are embedded in the current diff or comments. An older summary in `docs/frank_cuda_ba_benchmark_update.md` reports previous native CUDA BA timings, but it predates this graph-API benchmark mode.

### `--cuda-lm-graph`

`--cuda-lm-graph` runs the public graph optimizer wrapper (`timing/timeSFMBAL.cpp:551`). It builds the normal `GeneralSFMFactor` graph and initial `Values`, then times `CudaSfmLevenbergMarquardtOptimizer` construction plus `optimize()` inside `runCudaGraphLm` (`timing/timeSFMBAL.cpp:110`).

Compared with `--cuda-lm`, this mode includes:

- graph-to-`SfmData` conversion;
- explicit key preservation;
- optimized `Values` download and merge;
- final `graph.error()` recomputation for the optimizer state;
- reporting of graph API overhead over the backend measured total.

It does not include BAL parsing in the timed region. It also does not include the `buildGeneralSfmGraph` and `buildGeneralSfmInitial` calls for the measured dataset, because those happen before `runCudaGraphLm` starts its timer (`timing/timeSFMBAL.cpp:543`).

Example command structure:

```bash
./build-cuda-cudss-bench/timing/timeSFMBAL \
  --cuda-lm-graph \
  --cuda-linear-solver dense-schur \
  --cuda-warmup-file examples/Data/dubrovnik-3-7-pre.txt \
  examples/Data/dubrovnik-135-90642-pre.txt
```

The graph mode prints output shaped like:

```text
CUDA LM graph API: <seconds> s
CUDA LM graph linear solver: cudss-full-normal
CUDA LM graph API overhead over backend: <seconds> s
CUDA LM backend solve loop: <seconds> s
CUDA LM backend measured total: <seconds> s
...
Initial error: <value>
Final error: <value>, iterations: <accepted>, accepted: <accepted>, inner: <attempts>
```

A fresh A100 80GB run of the command above produced:

```text
CUDA graph warmup: 1.42539 s ignored
CUDA LM graph API: 0.41939 s
CUDA LM graph linear solver: dense-schur
CUDA LM graph API overhead over backend: 0.284713 s
CUDA LM backend solve loop: 0.0289198 s
CUDA LM backend measured total: 0.134678 s
download values: 0.0345623 s
Final error: 378041.329924042, iterations: 2, accepted: 2, inner: 3
```

The important interpretation is that `--cuda-lm-graph` is the user-facing
number: it times construction of the public CUDA optimizer plus `optimize()`.
The backend solve loop remains much smaller than the full graph API time
because conversion, packing/upload, download, merge, and final graph-error
bookkeeping are included around the solve.

### Warmup Semantics

`--cuda-warmup-file FILE` is accepted only with `--cuda-lm` or `--cuda-lm-graph` (`timing/timeSFMBAL.cpp:326`). Warmup runs once per process, before the first measured dataset, and its timing is explicitly printed as ignored (`timing/timeSFMBAL.cpp:443`, `timing/timeSFMBAL.cpp:553`).

The warmup mode uses the same CUDA solver selection as the measured run. For `--cuda-lm`, the warmup calls the low-level backend on `SfmData`. For `--cuda-lm-graph`, it builds a graph and initial values for the warmup file, then calls the graph optimizer wrapper.

### Option Restrictions

The CUDA LM modes are mutually exclusive with each other, `--cuda-structure-only`, and `--benchmark-action-json` (`timing/timeSFMBAL.cpp:305`). `--cuda-linear-solver` and `--cuda-warmup-file` are only valid when one CUDA LM mode is active (`timing/timeSFMBAL.cpp:323`). If CUDA or cuDSS support is missing, the binary reports that both CUDA LM modes require `GTSAM_ENABLE_CUDA=ON` and `GTSAM_ENABLE_CUDSS=ON` (`timing/timeSFMBAL.cpp:505`).

## Test Coverage

The CUDA SFM test additions cover the new behavior in `gtsam/slam/tests/testCudaSfm.cpp`:

- `ComputesClampedHessianDiagonal` checks the GPU Hessian diagonal against host accumulation and clamp logic (`gtsam/slam/tests/testCudaSfm.cpp:446`).
- `ComputesLinearizedErrorChange` verifies old/new linearized errors and predicted decrease (`gtsam/slam/tests/testCudaSfm.cpp:504`).
- `ConvertsGeneralSfmFactorsWithArbitraryKeys` validates graph conversion, arbitrary camera/point keys, measurements, cameras, and points (`gtsam/slam/tests/testCudaSfm.cpp:725`).
- `OptimizesGeneralSfmGraphWithArbitraryKeys` exercises the public optimizer wrapper, arbitrary symbols, retained unrelated values, and `innerIterations >= iterations` (`gtsam/slam/tests/testCudaSfm.cpp:774`).
- `ReducesTinyBalErrorAndDownloadsValues` still verifies that low-level CUDA LM reduces error and returns finite downloaded host values (`gtsam/slam/tests/testCudaSfm.cpp:821`).
- `CanSkipOptimizedValueDownload` covers benchmark-oriented metadata-only output (`gtsam/slam/tests/testCudaSfm.cpp:847`).
- `MatchesFullNormalEquationDeltaWithDiagonalDamping` checks that dense Schur and full-normal cuDSS deltas match when using the same clamped damping diagonal (`gtsam/slam/tests/testCudaSfm.cpp:904`).

Existing tests continue to cover CSR validation, cheirality behavior when enabled, mismatched value shapes, plain damping, and high-degree tracks.

## Current Caveats And Future Work

The most important caveat is scope: this is a CUDA BAL/SFM optimizer with a CPU-LM-shaped public wrapper, not a general CUDA `NonlinearFactorGraph` optimizer.

Current limitations:

- requires CUDA and cuDSS for optimization;
- supports only unit-noise `GeneralSFMFactor<SfmCamera, Point3>` graph conversion;
- does not support priors, robust models, weighted residuals, smart factors, separate calibration variables, fixed variables, or mixed variable types in the optimized CUDA state;
- `iterate()` is not implemented on the public optimizer;
- graph conversion and benchmark graph construction use existing BAL helpers whose track filtering semantics may differ from arbitrary user graphs;
- CUDA and CPU LM should be expected to have close but not identical trajectories because the linear solvers, lambda search details, and numerical reductions differ.

Likely future work:

- support whitening or weighted residuals so non-unit noise models can enter the CUDA backend;
- add prior/fixed-variable handling for graph API parity;
- reduce duplicate host-to-device work noted in `OptimizeCudaSfm`, where values and projection batches upload overlapping data (`gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu:310`);
- optimize `ComputeCudaSfmLinearizedErrorChange` to avoid repeated full linearization and host block-sum downloads during lambda retries;
- add benchmark harness support for repeats/medians for the new CUDA modes, rather than relying on external scripts;
- audit the adaptive non-fixed lambda path against CPU LM if exact `useFixedLambdaFactor=false` behavior becomes a compatibility requirement;
- expand tests from tiny BAL-like fixtures to larger real BAL files and failure-mode cases.
