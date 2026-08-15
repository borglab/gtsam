# CUDA Hybrid Solver: Upstream Merge and Rebenchmark

## Outcome

The final general hybrid CUDA solver commit, `458abdfbc`, was branched as
`codex/cuda-hybrid-final` and merged with upstream `develop` at
`49a9725f5`. The merge commit is `1ecee0c3c`; the tested branch tip is
`2d7902072` after one integration-test correction.

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
anchor prior. `timeCudaSparseLM` uses Ceres-style LM defaults and a unit prior,
which is why its raw correct-Jacobian Pose2 trajectory takes 32 accepted steps
rather than nine. The CPU 2x2 result establishes the Jacobian effect, but its
absolute iteration counts must not be transplanted directly to the hybrid
GPU harness. A follow-up GPU 2x2 should control those parameters and the prior
as well as initialization.

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
