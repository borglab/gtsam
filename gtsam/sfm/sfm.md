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

1. `SfmEliminationMode::Full` solves the joint camera–landmark system.
2. `SfmEliminationMode::Schur` eliminates landmarks first and solves the camera system.

CPU defaults to `Full`, `MULTIFRONTAL_SOLVER`, and an automatically generated natural-points/METIS-cameras ordering. That default won BAL-16 and BAL-88 and was within 0.3% of Schur on BAL-135. CUDA defaults to `Schur` and retains its CUDA backend selection.

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

## Measured CPU and CUDA performance

The CPU benchmark is [`timing/timeSfmPartialElimination.cpp`](../../timing/timeSfmPartialElimination.cpp). The CUDA matrix uses the public `gtsam::cuda::SfmLevenbergMarquardtOptimizer` path in [`timing/sfm_ba/timeCudaSFMBAL.cpp`](../../timing/sfm_ba/timeCudaSFMBAL.cpp); it does not restore the former `SfmFullNormalProblem` timing path or time a CPU fallback.

### Benchmark machine and build

Release-mode x86-64 measurements on August 21, 2026 used **Ubuntu 24.04.4 LTS with Linux 7.0.0-28-generic**. The machine had one Intel Core i7-14700F socket, 20 physical cores, 28 logical CPUs, one NUMA node containing CPUs 0–27, and 31 GiB RAM. Its NVIDIA GeForce RTX 5060 Ti had compute capability 12.0 and 16,311 MiB VRAM. The NVIDIA driver was 580.173.02.

The build used GNU C++ 13.3.0, CMake 4.2.3, CUDA toolkit 13.0.88 (`/usr/local/cuda-13.0/bin/nvcc`), cuDSS 0.8.0, CHOLMOD 5.3.1, and SuiteSparse_config 7.10.1. CMake detected the Ubuntu TBB 2021.11.0 headers/package; because the CHOLMOD conda directory precedes the system directory in the generated runpath, the measured executables loaded ABI-compatible TBB 2021.13.0 at runtime. CMake selected native CUDA architecture, the TBB allocator and automatic worker counts, system CHOLMOD, and `-O3 -DNDEBUG` for both C++ and CUDA Release compilation. The exact relevant configuration was:

```sh
cmake -S . -B build-cuda \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-13.0/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=native \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG' \
  '-DCMAKE_CUDA_FLAGS_RELEASE=-O3 -DNDEBUG' \
  -DGTSAM_ENABLE_CUDA=ON -DGTSAM_ENABLE_CUDSS=ON \
  -DCUDSS_ROOT=/home/dellaert/.local/opt/cudss-0.8.0-cuda13 \
  -DGTSAM_WITH_TBB=ON \
  -DTBB_DIR=/usr/lib/x86_64-linux-gnu/cmake/TBB \
  -DCHOLMOD_DIR=/home/dellaert/miniconda3/envs/gtsfm-v1/lib/cmake/CHOLMOD \
  -DSuiteSparse_config_DIR=/home/dellaert/miniconda3/envs/gtsfm-v1/lib/cmake/SuiteSparse_config \
  -DGTSAM_BUILD_TIMING_ALWAYS=ON -DGTSAM_BUILD_TESTS=ON \
  -DGTSAM_BUILD_DOCS=ON -DGTSAM_BUILD_PYTHON=OFF
cmake --build build-cuda --target \
  timeSfmPartialElimination timeCudaSFMBAL -j6
```

### Reproducing the CPU matrix

Each cell below came from three separate invocations of this form; `timeout` makes the five-minute limit a hard process boundary. Replace `CONFIGURATION` and `DATASET` with every row and BAL file shown in the matrix, calculate the median of the three reported single-run times, and stop that cell after exit status 124:

```sh
timeout 300s build-cuda/timing/timeSfmPartialElimination \
  --repetitions 1 --max-seconds 300 \
  --configuration 'CONFIGURATION' examples/Data/DATASET
```

The three datasets were `dubrovnik-16-22106-pre.txt`, `dubrovnik-88-64298-pre.txt`, and `dubrovnik-135-90642-pre.txt`. The configurations were the Full and Schur variants of `MultifrontalSolver`, `MultifrontalCholesky`, `MultifrontalQR`, `SequentialCholesky`, `SequentialQR`, `PCG`, `SubgraphSolver`, and `CHOLMOD`, using the names printed in the table with `/` in place of ` + `.

The program computes one ordering per dataset and reuses it everywhere: natural points followed by METIS cameras. Full receives the complete ordering; Schur receives its camera suffix and reconstructs the identical point prefix. PCG does not consume an elimination ordering. Data loading, ordering, and optimizer construction are excluded from the optimize time.

### Complete CPU timing matrix

Each value is the median of three complete nonlinear optimizations of the same point-batched BAL graph. Rows are sorted by completed BAL-135 time. `-` means an optimizer run reached the five-minute cap, `unsupported` means the public factor representation was rejected before a solve, and `unavailable` is reserved for an optional dependency that was not built. No dependency was unavailable on this machine.

| Configuration | BAL-16 s | BAL-88 s | BAL-135 s |
| --- | ---: | ---: | ---: |
| Schur + `MULTIFRONTAL_SOLVER` | 0.258 | 1.003 | **1.419** |
| Full + `MULTIFRONTAL_SOLVER` | **0.252** | **1.001** | 1.423 |
| Schur + multifrontal Cholesky | 0.422 | 1.378 | 2.009 |
| Schur + CHOLMOD | 0.454 | 1.429 | 2.035 |
| Schur + sequential Cholesky | 0.423 | 1.394 | 2.048 |
| Schur + PCG | 0.426 | 1.515 | 2.054 |
| Full + multifrontal Cholesky | 0.531 | 1.866 | 2.968 |
| Schur + multifrontal QR | 0.430 | 2.266 | 3.227 |
| Full + sequential Cholesky | 0.843 | 3.456 | 4.412 |
| Full + PCG | 0.658 | 3.826 | 4.646 |
| Full + CHOLMOD | 1.293 | 5.418 | 6.293 |
| Schur + sequential QR | 0.441 | 4.309 | 12.446 |
| Full + sequential QR | 3.813 | 121.922 | - |
| Full + multifrontal QR | 4.984 | 152.545 | - |
| Full + SubgraphSolver | unsupported¹ | unsupported¹ | unsupported¹ |
| Schur + SubgraphSolver | unsupported¹ | unsupported¹ | unsupported¹ |

¹ SubgraphSolver was attempted on all three datasets but failed before the first LM iteration. Its spanning-tree builder only uses unary and binary factors; the Full point-batched graph and the Schur camera clique graph contain higher-arity factors, so no spanning tree can be constructed.

Full multifrontal and sequential QR both crossed the five-minute cap on BAL-135. They remain supported, but are poor choices for this problem structure and hardware.

The fastest CPU configuration was not universal: Full + `MULTIFRONTAL_SOLVER` won BAL-16 and BAL-88, while Schur + `MULTIFRONTAL_SOLVER` won BAL-135 by about 0.25%. Both modes use the same point-first cached factorization and ordering.

### Reproducing the CUDA matrix

The CUDA runs used point-batched public factor graphs and `SfmEliminationMode::Schur`. Dense Cholesky and PCG used their required automatic ordering; cuDSS was measured with both automatic ordering and the accepted GTSAM camera ordering. Each command was run three separate times under the same hard cap:

```sh
build-cuda/timing/sfm_ba/timeCudaSFMBAL --list-configurations
timeout 300s build-cuda/timing/sfm_ba/timeCudaSFMBAL \
  --cuda-lm-graph --cuda-lm-graph-kind point-batch \
  --configuration CONFIGURATION --output-format json \
  examples/Data/DATASET
```

`CONFIGURATION` was each of `schur-dense`, `schur-cudss-auto`, `schur-cudss-gtsam`, and `schur-pcg`; `DATASET` was each of the three files above. Dataset loading and optimizer construction are excluded. The benchmark synchronizes CUDA immediately before and after the timed `optimize()` call.

### Complete CUDA timing matrix

These always-visible results are medians of three synchronized public-optimizer calls, sorted by BAL-135 time. The CUDA optimizer uses its bespoke CUDA kernels to form the landmark Schur complement; only the reduced camera solve changes between rows.

| Configuration | BAL-16 s | BAL-88 s | BAL-135 s |
| --- | ---: | ---: | ---: |
| Schur + dense Cholesky | **0.055** | **0.218** | **0.290** |
| Schur + cuDSS (automatic ordering) | 0.093 | 0.790 | 1.104 |
| Schur + cuDSS (GTSAM ordering) | 0.093 | 0.786 | 1.107 |
| Schur + PCG | 0.164 | 1.127 | 1.282 |

CUDA Full remains planned. It is exposed through `SfmEliminationMode::Full`, but optimizer construction/optimization rejects it with `CUDA SFM Full elimination mode is not implemented; select Schur.` No CPU fallback is timed as CUDA Full.

### Correctness checks

All completed repetitions had stable objectives and iteration counts. Full and Schur `MULTIFRONTAL_SOLVER` agreed at 18,033.914832 (5 LM/inner iterations), 291,581.106404 (4/4), and 377,782.056028 (2/2). Direct Full CPU solvers and the direct CUDA backends converged near 18,034.103054, 291,611.296974, and 378,041.329924; the CUDA accepted/inner counts were 5/5, 4/4, and 2/3. Explicit-Schur CPU direct solvers converged near 18,122.993769 (7/7), 293,146.832936 (4/4), and 378,183.228886 (3/3). Iterative endpoints were repeatable; the largest objective spread across all completed CPU configurations was 0.61%.

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
