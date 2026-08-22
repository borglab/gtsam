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

**Fastest measured:** `Full` + point-first ordering → `MULTIFRONTAL_SOLVER`

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

For BAL-style problems, use **point-batched projection factors + `Full` + `MULTIFRONTAL_SOLVER`** in a Release build with TBB enabled. Construct the complete ordering once with natural points followed by METIS cameras.

This is one cached, point-first multifrontal factorization. It does not materialize an intermediate reduced graph or invoke a second solver.
:::

## Two independent choices

`SfmLevenbergMarquardtOptimizer` separates the elimination strategy from the linear solver:

1. `SfmEliminationMode::Full` solves the joint camera–landmark system.
2. `SfmEliminationMode::Schur` eliminates landmarks first and solves the camera system.

CPU defaults to the fastest measured path: `Full`, `MULTIFRONTAL_SOLVER`, and an automatically generated natural-points/METIS-cameras ordering. CUDA defaults to `Schur` and retains its CUDA backend selection.

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
const gtsam::Ordering cameras =
    gtsam::SfmLevenbergMarquardtOptimizer::CreateCameraOrdering(
        graph, initial);
params.setOrdering(
    gtsam::SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(
        graph, cameras));

gtsam::SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
const gtsam::Values& result = optimizer.optimize();
```

## What Schur means for each solver

::::{tab-set}
:::{tab-item} MULTIFRONTAL_SOLVER

Schur mode constructs a complete ordering with a natural landmark prefix followed by the camera ordering. One cached `MultifrontalSolver` factorization eliminates the landmarks, factors the camera Schur complement, and back-substitutes through the same Bayes tree.

No reduced factor graph is created. This is mathematically the same point-first elimination used in Full mode when it receives the identical complete ordering; the explicit `Schur` mode lets SFM construct that ordering from a camera-only input.
:::

:::{tab-item} Other CPU solvers

Schur mode creates an explicit solver boundary:

1. `MultifrontalSolver::eliminatePartialInPlace()` eliminates landmarks.
2. `remainingFactorGraph()` exports the camera Hessian graph.
3. The selected ordinary solver computes the camera step.
4. `updateSolution(cameraDelta)` back-substitutes the landmark step.

The partial-elimination symbolic state is reused across LM attempts and iterations. This path lets CHOLMOD, legacy Cholesky or QR, and iterative solvers operate on the reduced graph, at the cost of materializing camera factors and running a separate solve.
:::
::::

### Camera-only ordering

In Full mode, a supplied ordering has ordinary all-variable semantics. In Schur mode, supply only camera keys:

```cpp
gtsam::Ordering cameras{camera0Key, camera1Key, camera2Key};
params.setOrdering(cameras);
```

`CreateCameraOrdering(graph, initial)` symbolically eliminates the natural point ordering and runs METIS on the resulting camera graph. The optimizer prefixes that camera ordering with active non-camera keys in natural key order; `CreatePointFirstOrdering(graph, cameras)` exposes the identical complete ordering when Full mode or benchmarking needs it. Duplicate, missing, landmark, and unknown camera keys are rejected. Direct reduced solvers and iterative solvers that consume an ordering receive the same camera-only suffix.

::::{grid} 1 1 2 2

:::{card} Iterative dispatch

Select `NonlinearOptimizerParams::Iterative` and supply the same parameter object as an ordinary nonlinear optimizer:

- `PCGSolverParameters` selects PCG.
- `SubgraphSolverParameters` selects `SubgraphSolver` and receives the camera ordering in Schur mode.
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

## Measured CPU performance

The public benchmark is [`timing/timeSfmPartialElimination.cpp`](../../timing/timeSfmPartialElimination.cpp), while [`timing/timeSFMBAL.cpp`](../../timing/timeSFMBAL.cpp) configures the public optimizer for the main BAL comparison. They replace the former timing-only compact Schur and camera-specific CHOLMOD implementations.

Run the complete CPU mode/solver matrix with a five-minute cap per run:

```sh
./timing/timeSfmPartialElimination --repetitions 3 --max-seconds 300 \
  /path/to/problem-135-90642-pre.txt
```

The program computes one ordering per dataset and reuses it everywhere: natural points followed by METIS cameras. Full receives the complete ordering; Schur receives its camera suffix and reconstructs the identical point prefix. PCG does not consume an elimination ordering. Optimizer construction and ordering are excluded from the optimize time. A run reaching five minutes is reported as `-` and its remaining repetitions are skipped.

### Benchmark machine and complete timing matrix

Release-mode arm64 measurements on August 21, 2026 used macOS 26.5.2 on a MacBook Air (Mac17,4) with an **Apple M5, 10 CPU cores (4 performance + 6 efficiency), and 24 GB memory**. The build used Apple Clang 22.1.7, `-O3 -DNDEBUG`, TBB 2022.3.0, and CHOLMOD 5.3.4. `MULTIFRONTAL_SOLVER` and PCG used automatic worker counts; legacy solver parallelism was unchanged. Each value is the median of three complete nonlinear optimizations of a point-batched BAL graph.

| Configuration | BAL-16 s | BAL-88 s | BAL-135 s |
| --- | ---: | ---: | ---: |
| Full + `MULTIFRONTAL_SOLVER` | **0.228** | **0.931** | **1.300** |
| Schur + `MULTIFRONTAL_SOLVER` | 0.231 | 0.955 | 1.374 |
| Schur + sequential Cholesky | 0.405 | 1.360 | 1.930 |
| Schur + multifrontal Cholesky | 0.378 | 1.296 | 1.943 |
| Schur + PCG | 0.386 | 1.379 | 2.665 |
| Schur + CHOLMOD | 0.711 | 2.060 | 2.780 |
| Schur + multifrontal QR | 0.383 | 2.206 | 3.200 |
| Full + sequential Cholesky | 0.640 | 3.684 | 4.432 |
| Full + PCG | 0.536 | 4.119 | 5.413 |
| Full + CHOLMOD | 1.373 | 3.807 | 7.865 |
| Full + multifrontal Cholesky | 0.637 | 1.973 | 8.102 |
| Schur + sequential QR | 0.417 | 3.355 | 8.273 |
| Full + sequential QR | 2.361 | 74.741 | 203.216 |
| Full + multifrontal QR | 2.457 | 84.486 | - |
| Full + SubgraphSolver | unsupported¹ | unsupported¹ | unsupported¹ |
| Schur + SubgraphSolver | unsupported¹ | unsupported¹ | unsupported¹ |

¹ SubgraphSolver was attempted on all three datasets but failed before the first LM iteration. Its spanning-tree builder only uses unary and binary factors; the Full point-batched graph and the Schur camera clique graph contain higher-arity factors, so no spanning tree can be constructed.

The BAL-135 Full QR phase reached a 22.4 GB process peak on this 24 GB machine. Full multifrontal QR crossed the five-minute cap; Full sequential QR completed but took more than three minutes per optimization. These combinations are supported, but poor choices for this problem structure and hardware.

These are end-to-end optimizer timings, not fixed reduced-system microbenchmarks. Numerical differences between fused and explicitly materialized systems can alter LM trajectories and iteration counts; compare final objectives and iteration counts as well as elapsed time.

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
