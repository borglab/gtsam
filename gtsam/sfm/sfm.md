---
title: Structure from Motion
subtitle: Bundle adjustment, elimination strategy, and CPU/CUDA solver selection
description: User guides and performance guidance for GTSAM structure from motion.
thumbnail: ./doc/images/cuda-optimizers.png
---

GTSAM's `sfm` module spans the full reconstruction pipeline: feature tracks, global rotation and translation recovery, trajectory alignment, and high-performance CPU and CUDA bundle adjustment.

```{figure} doc/images/cuda-optimizers.png
:name: fig-sfm-cuda-optimizers
:alt: Architecture of GTSAM's general and structure-from-motion CUDA Levenberg-Marquardt optimizers
:align: center
:class: sfm-hero

The general and SFM Levenberg–Marquardt paths on the GPU. The SFM optimizer uses bespoke CUDA kernels to form its camera Schur complement before dispatching to dense Cholesky, cuDSS, or PCG.
```

::::{grid} 1 1 2 2
:class: sfm-capability-grid

:::{card} CPU bundle adjustment

**Full or Schur** · standard nonlinear solver selection

Use multifrontal or sequential Cholesky/QR, `MULTIFRONTAL_SOLVER`, iterative PCG/SubgraphSolver, or optional CHOLMOD.

**Fastest measured family:** point-first `MULTIFRONTAL_SOLVER` (`Full` or `Schur`)

+++
[Open the CPU guide](doc/SfmLevenbergMarquardtOptimizer.ipynb)
:::

:::{card} CUDA bundle adjustment

**Schur today** · Full mode reserved

The bespoke CUDA Schur path supports dense Cholesky, cuDSS, and PCG. `Full` is already part of the public API but throws a clear not-implemented exception.

+++
[Open the CUDA guide](doc/CudaSfmLevenbergMarquardtOptimizer.ipynb) · [Use GNC with CUDA SFM](doc/CudaSfmGncOptimizer.ipynb)
:::
::::

:::{admonition} Fastest tested CPU path
:class: tip sfm-fast-path

For BAL-style problems, use **point-batched projection factors + `MULTIFRONTAL_SOLVER`** in a Release build with TBB enabled. Construct the complete ordering once with natural points followed by METIS cameras. `Full` was fastest on BAL-16 and BAL-88, while `Schur` was fastest on BAL-135, so these measurements do not identify one universally fastest elimination mode.

Both modes use one cached, point-first multifrontal factorization. They do not materialize an intermediate reduced graph or invoke a second solver.
:::

## Two independent choices

`SfmLevenbergMarquardtOptimizer` separates the elimination strategy from the linear solver:

1. `SfmEliminationMode::Full` solves the joint system.
2. `SfmEliminationMode::Schur` eliminates every active `Point3` and `Unit3` first, then solves for all remaining variables.

On CPU, the reduced system is selected by value type rather than by one camera template: poses, camera objects, shared or per-camera calibrations, and any other value types remain after Schur elimination. This supports variable and global calibration without specializing the optimizer. CPU defaults to `Full`, `MULTIFRONTAL_SOLVER`, and an automatically generated natural-landmark prefix followed by a METIS ordering of the reduced system. That default won BAL-16 and BAL-88 and was within 0.3% of Schur on BAL-135. CUDA defaults to `Schur` and keeps its existing camera/`Point3` backend assumptions.

| Platform | Full | Schur | Linear solvers |
| --- | --- | --- | --- |
| **CPU** | **Supported (default)** | Supported | **`MULTIFRONTAL_SOLVER` (default)**, multifrontal/sequential Cholesky or QR, `Iterative`, optional `CHOLMOD` |
| **CUDA** | Planned; currently throws | **Supported (default)** | dense Cholesky, cuDSS, PCG |

### CPU quick start

```cpp
#include <gtsam/sfm/SfmLevenbergMarquardt.h>

gtsam::SfmLevenbergMarquardtParams params =
    gtsam::SfmLevenbergMarquardtParams::ceresDefaults();

// For the fastest measured path, graph uses point-batched projection factors.
const gtsam::Ordering reducedOrdering =
    gtsam::SfmLevenbergMarquardtOptimizer::CreateReducedOrdering(
        graph, initial);
params.setOrdering(
    gtsam::SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(
        graph, reducedOrdering));

gtsam::SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
const gtsam::Values& result = optimizer.optimize();
```

[Run the Python Full-and-Schur tutorial](../../python/gtsam/examples/SfmLevenbergMarquardtOptimizerExample.ipynb),
including a calibration shared by every camera. When constructing `Values` from
NumPy arrays, use `Values.insertPoint3()` for landmarks so Python preserves
their fixed-size `Point3` type.

## What Schur means for each solver

::::{tab-set}
:::{tab-item} MULTIFRONTAL_SOLVER

Schur mode constructs a complete ordering with a natural `Point3`/`Unit3` prefix followed by the reduced-system ordering. One cached `MultifrontalSolver` factorization eliminates those landmarks, factors the Schur complement over the remaining variables, and back-substitutes through the same Bayes tree.

No reduced factor graph is created. This is mathematically the same landmark-first elimination used in Full mode when it receives the identical complete ordering; explicit `Schur` mode lets SFM construct that ordering from a reduced-system input.
:::

:::{tab-item} Other CPU solvers

Schur mode creates an explicit solver boundary:

1. `MultifrontalSolver::eliminatePartialInPlace()` eliminates landmarks.
2. `remainingFactorGraph()` exports the reduced Hessian graph.
3. The selected ordinary solver computes the reduced-system step.
4. `updateSolution(reducedDelta)` back-substitutes the eliminated step.

The partial-elimination symbolic state is reused across LM attempts and iterations. This path lets CHOLMOD, legacy Cholesky or QR, and iterative solvers operate on the reduced graph, at the cost of materializing reduced factors and running a separate solve.
:::
::::

### Reduced-system ordering

In Full mode, a supplied ordering has ordinary all-variable semantics. In CPU Schur mode, supply every active key whose value is neither `Point3` nor `Unit3`:

```cpp
gtsam::Ordering reduced{pose0Key, pose1Key, globalCalibrationKey};
params.setOrdering(reduced);
```

`CreateReducedOrdering(graph, initial)` symbolically eliminates active `Point3` and `Unit3` values in natural key order and runs METIS on the resulting reduced graph. `CreateSchurOrdering(graph, reduced)` prefixes that reduced ordering with the eliminated keys; its result is the complete Schur ordering used by Full mode or the fused multifrontal path. Duplicate, missing, eliminated, and unknown reduced keys are rejected. Direct reduced solvers and iterative solvers that consume an ordering receive the same reduced-only suffix.

::::{grid} 1 1 2 2

:::{card} Iterative dispatch

Select `NonlinearOptimizerParams::Iterative` and supply the same parameter object as an ordinary nonlinear optimizer:

- `PCGSolverParameters` selects PCG.
- `SubgraphSolverParameters` selects `SubgraphSolver` and receives the reduced-system ordering in Schur mode.
- Missing or unrecognized parameters throw the ordinary `NonlinearOptimizer::solve` error.

`SubgraphSolver` additionally requires a Gaussian graph with enough unary and binary factors to construct its spanning-tree preconditioner. Higher-arity point batches and camera clique factors do not satisfy that requirement.

The typed `getLinearSolver()` and `setLinearSolver()` conveniences coexist with the compatible string APIs.
:::

:::{card} Optional CHOLMOD

When SuiteSparse CHOLMOD is found, `NonlinearOptimizerParams::CHOLMOD` works for ordinary nonlinear optimizers and both CPU SFM modes. The reusable session accepts arbitrary variable dimensions, loads ordinary Jacobian, compact batch Jacobian, and Hessian factors directly into the sparse normal system, expands key orderings into scalar permutations, and reuses symbolic analysis. Compact batches contribute their fixed-size normal blocks without a dense intermediate.

A build without CHOLMOD remains valid; selecting it produces an actionable runtime error. Constrained, non-SPD, and factorization failures are reported rather than silently falling back.
:::
::::

### CUDA semantics

CUDA Schur mode retains the bespoke landmark-elimination kernels shown in {numref}`fig-sfm-cuda-optimizers`. Dense Cholesky, cuDSS, and PCG solve the resulting camera system. CUDA orderings remain camera-only and are accepted only by backends that consume them. Selecting `SfmEliminationMode::Full` fails immediately; the full-system CUDA implementation is intentionally deferred.

## Performance at a glance

These Release-mode measurements were collected on Ubuntu 24.04 with an **Intel Core i7-14700F (20 physical cores, 28 logical CPUs) and 31 GiB RAM**, plus an **NVIDIA GeForce RTX 5060 Ti with 16 GiB VRAM**. Values are median end-to-end optimization times from three runs.

| Public optimizer path | BAL-16 s | BAL-88 s | BAL-135 s |
| --- | ---: | ---: | ---: |
| CPU Full + `MULTIFRONTAL_SOLVER` | **0.252** | **1.001** | 1.423 |
| CPU Schur + `MULTIFRONTAL_SOLVER` | 0.258 | 1.003 | **1.419** |
| CUDA Schur + dense Cholesky | **0.055** | **0.218** | **0.290** |

:::{admonition} Fast-path guidance
:class: tip

For CPU BAL-style problems, start with point-batched projection factors, the natural-points/METIS-cameras ordering, and `MULTIFRONTAL_SOLVER`. Full is the default and won two datasets; Schur was effectively tied and won the largest by only 0.25%. This benchmark does not measure parallel scaling, so physical core count alone is not a processor-performance prediction.

For CUDA at these problem sizes, use Schur with dense Cholesky. It was about 4.6–4.9× faster than the best CPU result. CUDA Full remains planned and is rejected rather than silently falling back to the CPU.
:::

The [timing benchmark README](../../timing/README.md#public-sfm-solver-matrix) contains the complete solver matrices, build record, commands, timeout/unsupported results, and numerical checks.

## User guides by class

::::{grid} 1 1 2 2
:class: sfm-guide-grid

:::{card} Data and feature tracks

- [Keypoints](doc/Keypoints.ipynb)
- [SfmTrack2d](doc/SfmTrack2d.ipynb)
- [SfmTrack](doc/SfmTrack.ipynb)
- [SfmData](doc/SfmData.ipynb)
:::

:::{card} Measurement and transfer factors

- [UnaryMeasurement](doc/UnaryMeasurement.ipynb)
- [BinaryMeasurement](doc/BinaryMeasurement.ipynb)
- [TransferEdges](doc/TransferEdges.ipynb)
- [TransferFactor](doc/TransferFactor.ipynb)
- [EssentialTransferFactor](doc/EssentialTransferFactor.ipynb)
- [EssentialTransferFactorK](doc/EssentialTransferFactorK.ipynb)
- [SelfCalibrationFactor](doc/SelfCalibrationFactor.ipynb)
:::

:::{card} Rotation averaging

- [ShonanFactor](doc/ShonanFactor.ipynb)
- [ShonanGaugeFactor](doc/ShonanGaugeFactor.ipynb)
- [ShonanAveragingParameters](doc/ShonanAveragingParameters.ipynb)
- [ShonanAveraging](doc/ShonanAveraging.ipynb)
- [ShonanAveraging2](doc/ShonanAveraging2.ipynb)
- [ShonanAveraging3](doc/ShonanAveraging3.ipynb)
:::

:::{card} Translation and position recovery

- [TranslationFactor](doc/TranslationFactor.ipynb)
- [BilinearAngleTranslationFactor](doc/BilinearAngleTranslationFactor.ipynb)
- [LocationRecovery](doc/LocationRecovery.ipynb)
- [TranslationRecovery](doc/TranslationRecovery.ipynb)
- [GlobalPositioner](doc/GlobalPositioner.ipynb)
- [MFAS](doc/MFAS.ipynb)
:::

:::{card} Trajectory alignment

- [TrajectoryAlignerSim3](doc/TrajectoryAlignerSim3.ipynb)
:::
::::

Template classes appear in Python under concrete names. Wrapper-only containers adapt C++ containers at language boundaries; Python users should normally pass native lists and dictionaries as shown in the related guides.
