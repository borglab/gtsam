# CUDA Hybrid Solver: Upstream Merge and Rebenchmark

## Outcome

The final general hybrid CUDA solver commit, `458abdfbc`, was branched as
`codex/cuda-hybrid-final` and merged with upstream `develop` at
`49a9725f5`. The merge commit is `1ecee0c3c`; the solver integration tip is
`2d7902072` after one integration-test correction. The controlled GPU 2x2
below was recorded at `7196db0d9`, after benchmark-only additions for FastSync
and upstream-matched Pose2 settings.

This is the intended hybrid baseline. It contains
`CudaSparseLevenbergMarquardt`, where nonlinear factor linearization and
`Values::retract` remain on the CPU and the sparse linear solve runs on the
GPU. It does not contain the later fully GPU-resident nonlinear optimizer
(`CudaNonlinearOptimizerEngine`, `CudaGraphCompiler`, CUDA factor backends,
or the general CUDA linear-system backend).

Upstream PR #2661, "Enable correct Lie-group Jacobians in Prior/Between by
default," is an ancestor of the merged branch. A reused CMake cache initially
kept the old option value, so the valid post-merge results below were produced
only after explicitly configuring:

```text
-DGTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR=ON
```

The earlier files under `benchmarks/postmerge/` used the stale `OFF` cache and
must not be used as the post-merge comparison.

## Environment and method

| Item | Configuration |
| --- | --- |
| GPU | NVIDIA A100 80GB PCIe, compute capability 8.0 |
| Driver | 580.173.02 |
| CUDA | 13.0 |
| Build | Release, CUDA and cuDSS enabled, TBB enabled |
| Baseline | `458abdfbc`, legacy Prior/Between Jacobians |
| Merged | `2d7902072`, upstream default local Jacobians enabled |
| Samples | One warm-up, five measured repetitions |
| Workloads | Dubrovnik-16, Dubrovnik-135, `w10000`, `sphere_smallnoise` |

Raw JSON and CSV are in the ignored build tree:

```text
build-cuda-cudss-on/benchmarks/premerge/
build-cuda-cudss-on/benchmarks/postmerge-correct-jacobians/
build-cuda-cudss-on/benchmarks/gpu-2x2/
build-cuda-cudss-on/benchmarks/gpu-2x2-matched/
build-cuda-cudss-legacy-jacobian/benchmarks/gpu-2x2/
build-cuda-cudss-legacy-jacobian/benchmarks/gpu-2x2-matched/
```

## Correctness-valid cuDSS comparison

Times are five-run end-to-end medians. Delta is merged GPU wall time relative
to the pre-merge baseline; negative is faster.

| Workload | GPU before | GPU merged | GPU delta | Merged CPU/GPU | LM attempts before -> merged | CPU objective before -> merged |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Dubrovnik-16 | 0.669 s | 0.683 s | +2.1% | 3.70x | 5 -> 5 | 18,034 -> 18,034 |
| Dubrovnik-135 | 3.503 s | 3.519 s | +0.5% | 1.97x | 3 -> 3 | 378,041 -> 378,041 |
| Pose2 `w10000` (raw initialization) | 0.344 s | 1.119 s | +224.9% | 3.93x | 11 -> 34 | 5,572,632 -> 162.034 |
| Pose3 `sphere_smallnoise` | 0.720 s | 0.749 s | +3.9% | 4.61x | 54 -> 53 | 5,037 -> 5,232 |

Every cuDSS run passed the strict `1e-8` CPU/GPU objective-parity check. The
BAL workloads do not use Prior/Between pose-graph factors, so their trajectory
and GPU time remain essentially unchanged. Pose3 hit the 50-iteration limit
both before and after, so its different objective is a trajectory-at-fixed-
budget result, not a converged-solution comparison.

The raw Pose2 result changes qualitatively. The legacy approximate Jacobians
stopped quickly at a poor objective of 5.57 million. Correct Jacobians reach
162.034, but from the raw 24.72-million-error initialization they perform 34
linear solves and take 1.119 seconds. The merged solver is still 3.93x faster
than CPU for that much more accurate solve.

## PCG comparison and accuracy limit

Default block-Jacobi PCG remains correctness-valid on both BAL workloads and
Pose3:

| Workload | GPU before | GPU merged | GPU delta | Merged CPU/GPU | LM attempts before -> merged |
| --- | ---: | ---: | ---: | ---: | ---: |
| Dubrovnik-16 | 0.420 s | 0.394 s | -6.1% | 6.21x | 5 -> 5 |
| Dubrovnik-135 | 1.371 s | 1.342 s | -2.1% | 5.38x | 3 -> 3 |
| Pose3 `sphere_smallnoise` | 0.741 s | 0.724 s | -2.3% | 5.13x | 54 -> 53 |

Default PCG is not accurate enough for the newly improved raw Pose2 solution.
The normal 2% parity run aborts because CPU reaches 162.034 while GPU reaches
253.088, a 56.2% objective error. A deliberately loose 100% tolerance was
used only to save a five-run control: 1.529 s GPU, 2.92x over CPU, 47 LM
attempts. That timing is not a correctness-valid result.

Tightening PCG to `--pcg-tol 1e-8 --pcg-max-iters 1000` restores parity:

| Pose2 configuration | GPU median | CPU/GPU | LM attempts | GPU objective | Status |
| --- | ---: | ---: | ---: | ---: | --- |
| Default PCG control | 1.529 s | 2.92x | 47 | 253.088 | Fails 2% parity |
| Tight PCG | 1.824 s | 2.45x | 33 | 162.069 | Passes 2% parity |
| cuDSS | 1.119 s | 3.93x | 34 | 162.034 | Passes strict parity |

For raw `w10000` with correct Jacobians, cuDSS is therefore both faster and
more accurate than the current PCG configuration. PCG needs further
preconditioning/convergence work before it should be the default for this
new trajectory.

## Two-by-two initialization/Jacobian isolation

Upstream's CPU-only `timeBetweenFactorPoseGraph` benchmark was run with both
raw and FastSync initialization. Within each initialization row it compares
the real `BetweenFactor` against a legacy-factor implementation, so graph,
starting `Values`, LM settings, and measured-run order are held constant.

| Initialization | Jacobians | Initial objective | LM median | Inner LM attempts | Final objective |
| --- | --- | ---: | ---: | ---: | ---: |
| Raw | Legacy approximation | 24,720,120 | 3.750 s | 28 | 4,642,335 |
| Raw | Correct local | 24,720,120 | 1.161 s | 9 | 144.863 |
| FastSync | Legacy approximation | 23,454 | 6.201 s | 42 | 144.877 |
| FastSync | Correct local | 23,454 | 1.786 s | 17 | 144.910 |

The correct-Jacobian LM speedup is **3.23x from raw values** and **3.47x from
FastSync values**. Frank's 42 -> 17 result is reproduced, but the raw 28 -> 9
result shows that the Jacobian improvement does not depend on FastSync. The
raw legacy run also terminates at a catastrophically worse objective, while
the correct run reaches the same approximately 145 objective as the
FastSync-initialized runs.

FastSync itself took 0.648 seconds. Including that cost, its correct-Jacobian
pipeline took 2.434 seconds, versus 1.161 seconds for correct-Jacobian LM from
the raw dataset values. Thus FastSync was 2.10x slower end-to-end on this
particular graph and configuration despite reducing the initial objective by
roughly three orders of magnitude. Initial objective magnitude alone does not
predict LM trajectory length.

This isolation benchmark uses upstream's default LM parameters and tight
anchor prior. The hybrid harness normally uses Ceres-style LM defaults and a
unit prior, which is why its raw correct-Jacobian Pose2 trajectory takes 34
lambda attempts rather than nine. The next two sections separate that settings
effect from the Jacobian effect on GPU.

## GPU 2x2 with native hybrid settings

As a diagnostic control, the hybrid solver was first run with its existing
Ceres-style LM defaults and unit anchor prior. All times are GPU wall medians
from one warm-up and five measured repetitions using cuDSS.

| Initialization | Jacobians | Initialization | GPU LM | LM attempts | Final objective |
| --- | --- | ---: | ---: | ---: | ---: |
| Raw | Legacy approximation | 0 s | 0.373 s | 11 | 5,572,632 |
| Raw | Correct local | 0 s | 1.155 s | 34 | 162.034 |
| FastSync | Legacy approximation | 0.551 s | 0.463 s | 13 | 301.802 |
| FastSync | Correct local | 0.632 s | 1.473 s | 44 | 175.762 |

Under these settings the legacy solver appears faster only because it stops
much earlier at a substantially worse objective. This is not a speed-to-equal-
solution comparison. It also explains why the initial hybrid run seemed to
contradict Frank's result: the Jacobian switch changed the LM trajectory while
the benchmark settings differed from upstream.

## GPU 2x2 with upstream-matched settings

The decisive run uses the tight prior sigmas `(1e-6, 1e-6, 1e-8)` and GTSAM's
default LM parameters, matching `timeBetweenFactorPoseGraph`. The only two
variables are initialization and the compile-time BetweenFactor Jacobian
mode. Each cell is one warm-up plus five measured repetitions on the A100 with
cuDSS. Every result came from clean commit `7196db0d9` and passed the `1e-8`
CPU/GPU objective-parity check.

| Initialization | Jacobians | Initialization | GPU LM median | CPU LM median | CPU/GPU | LM attempts | Final objective |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Raw | Legacy approximation | 0 s | 0.636 s | 2.531 s | 3.98x | 28 | 4,642,335.229 |
| Raw | Correct local | 0 s | 0.402 s | 1.054 s | 2.62x | 9 | 144.863 |
| FastSync | Legacy approximation | 0.661 s | 1.120 s | 4.169 s | 3.72x | 42 | 144.877 |
| FastSync | Correct local | 0.564 s | 0.490 s | 1.650 s | 3.37x | 17 | 144.910 |

Correct Jacobians make GPU LM **1.58x faster from raw values** and **2.28x
faster after FastSync**. Therefore the Jacobian benefit does not depend on
FastSync. From raw values it also changes the outcome from a lambda-upper-bound
termination at a 4.64-million objective to convergence at 144.863.

FastSync is opt-in, not the default. It lowers the initial objective from
24.72 million to 23,454, but it does not shorten the correct-Jacobian solve on
this graph: GPU LM rises from 0.402 to 0.490 seconds, before paying another
0.564 seconds for initialization. The fastest accurate pipeline in this matrix
is therefore correct Jacobians from the raw dataset values. FastSync is what
makes the legacy run accurate here, but that pipeline costs 1.780 seconds
including initialization, versus 0.402 seconds for raw plus correct Jacobians.

## Integration validation

- `timeCudaSparseLM`, `timeCudaSFMBAL`, and
  `timeBetweenFactorPoseGraph` build successfully.
- `testCudaSparseJacobian` passes.
- `testCudaSparseLevenbergMarquardt` passes.
- `timeCudaSparseLM --self-test` passes.
- The SFM benchmark retains the hybrid sparse-LM and GNC command-line modes
  after resolving its upstream API conflict.

The LM integration test previously required CPU and cuDSS to accept exactly
the same number of steps. Correct local Jacobians exposed numerically
borderline trials where the two direct solvers take different paths but still
match final objective, final `Values`, and every streamed Jacobian snapshot.
The test now checks valid CUDA progress while retaining those stronger
correctness assertions.
