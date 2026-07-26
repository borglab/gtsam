# CUDA Direct-Sparse LM Multidomain Benchmark

## Outcome

The generic direct-sparse CUDA Levenberg-Marquardt optimizer was faster than
ordinary CPU GTSAM LM on all four requested workloads. Across five measured
runs after one untimed warm-up, median end-to-end speedups ranged from 1.79x
on Dubrovnik-135 to 5.23x on the Pose3 graph.

| Workload | CPU median | GPU median | CPU / GPU |
| --- | ---: | ---: | ---: |
| Dubrovnik-16 | 2.095074 s | 0.677193 s | 3.09x |
| Dubrovnik-135 | 5.701630 s | 3.186185 s | 1.79x |
| Pose2 `w10000` | 1.435487 s | 0.339619 s | 4.23x |
| Pose3 `sphere_smallnoise` | 3.607331 s | 0.690003 s | 5.23x |

Every measured CPU and GPU run passed the benchmark's relative objective
parity check. The maximum absolute CPU/GPU difference was
`1.5832e-7`; relative to the Pose2 final objective of `5.5726e6`, this
is approximately `2.8e-14`.

## Artifacts

- `timing/cuda_sparse/results/2026-07-23-a100/benchmark.json` contains all
  40 measured runs, complete GPU attempt traces, transfer counts, all 28
  detailed non-alias stage fields, and five-sample summary statistics.
- `timing/cuda_sparse/results/2026-07-23-a100/summary.csv` contains one summary
  row per workload/backend.
- `timing/cuda_sparse/results/2026-07-23-a100/console.log` is the complete
  benchmark transcript.

## Environment

| Item | Configuration |
| --- | --- |
| GPU | NVIDIA A100 80GB PCIe, compute capability 8.0 |
| GPU memory | 80 GiB |
| NVIDIA driver | 580.159.03 |
| CUDA toolkit/runtime | 13.0.88 / 13.0 |
| cuDSS | 0.8.0.10 |
| CPU | AMD EPYC-Milan Processor, 32 logical CPUs |
| CPU parallelism | TBB enabled |
| Build | Release, `-O3 -DNDEBUG` |
| Branch | `codex/direct-sparse-jacobian` |
| Base revision | `df77dfaf1326cb7706bcbbe01beda82823d96089` |
| Working tree | Dirty because the new benchmark and its results were not committed |
| Date | 2026-07-23 |

The exact command was:

```text
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 1 --repeats 5 \
  --datasets bal16,bal135,pose2,pose3 \
  --data-dir /home/ubuntu/cu-gtsam/examples/Data \
  --json timing/cuda_sparse/results/2026-07-23-a100/benchmark.json \
  --csv timing/cuda_sparse/results/2026-07-23-a100/summary.csv
```

## Fairness and workload construction

Each backend received the same `NonlinearFactorGraph`, initial `Values`, noise
models, gauge prior, and nonlinear LM settings. Each measured run constructed
a fresh optimizer and timed construction plus `optimize()`. One CPU and one
GPU warm-up were excluded. Measured backend order alternated CPU/GPU,
GPU/CPU, CPU/GPU, GPU/CPU, CPU/GPU.

CPU used ordinary `LevenbergMarquardtOptimizer`; GPU used
`CudaSparseLevenbergMarquardtOptimizer` with CPU fallback disabled. The
benchmark failed immediately if the GPU path fell back, produced non-finite
timing or attempt data, omitted a termination reason, or violated objective
parity. BAL used the existing SFM benchmark's Schur ordering on CPU; cuDSS
retained control of sparse ordering on GPU.

Pose2 and Pose3 each received a unit-noise prior on pose 0 to remove gauge
freedom. `sphere_smallnoise.graph` uses legacy TORO `EDGE3` records that do
not contain an information matrix, unlike the records accepted by GTSAM's
generic `load3D()`. The benchmark therefore parses all 8,647 relative-pose
records explicitly and assigns the same unit 6-D noise model to every edge
for both CPU and GPU.

The parity tolerance was:

```text
1e-8 * max(1, abs(reference CPU objective), abs(candidate objective))
```

## Problem sizes and correctness

| Workload | Factors | Values | Residual rows | Columns | `J.nnz` | `H.nnz` | Initial objective | Final objective | Maximum difference |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Dubrovnik-16 | 83,718 | 22,122 | 167,436 | 66,462 | 2,009,232 | 4,721,022 | 4,185,659.518 | 18,034.103054 | `1.346e-10` |
| Dubrovnik-135 | 553,336 | 90,777 | 1,106,672 | 273,141 | 13,280,064 | 30,706,857 | 57,945,241.702 | 378,041.329924 | `6.158e-8` |
| Pose2 | 64,312 | 10,000 | 192,936 | 30,000 | 1,157,607 | 1,247,598 | 24,720,119.960 | 5,572,631.532514 | `1.583e-7` |
| Pose3 | 8,648 | 2,200 | 51,888 | 13,200 | 622,620 | 701,784 | 496,316.127 | 5,037.069214 | `2.499e-9` |

## Five-run end-to-end statistics

Times are construction plus optimization wall time. Standard deviation is the
population standard deviation of the five measured samples.

| Workload | Backend | Median | Mean | Stddev | Minimum | Maximum |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Dubrovnik-16 | CPU | 2.095074 s | 2.100851 s | 0.013848 s | 2.086599 s | 2.124491 s |
| Dubrovnik-16 | GPU | 0.677193 s | 0.676924 s | 0.004260 s | 0.669409 s | 0.681249 s |
| Dubrovnik-135 | CPU | 5.701630 s | 5.787284 s | 0.353266 s | 5.481556 s | 6.466086 s |
| Dubrovnik-135 | GPU | 3.186185 s | 3.239563 s | 0.112141 s | 3.166466 s | 3.462601 s |
| Pose2 | CPU | 1.435487 s | 1.416699 s | 0.055290 s | 1.318896 s | 1.478754 s |
| Pose2 | GPU | 0.339619 s | 0.336201 s | 0.007283 s | 0.323311 s | 0.343897 s |
| Pose3 | CPU | 3.607331 s | 3.605447 s | 0.017979 s | 3.580642 s | 3.634967 s |
| Pose3 | GPU | 0.690003 s | 0.685115 s | 0.009765 s | 0.671433 s | 0.696054 s |

The GPU termination records were:

| Workload | Iterations | Outer linearizations | Lambda attempts | cuDSS analyses | Termination |
| --- | ---: | ---: | ---: | ---: | --- |
| Dubrovnik-16 | 5 | 5 | 5 | 1 | converged |
| Dubrovnik-135 | 2 | 3 | 3 | 1 | small cost change |
| Pose2 | 5 | 6 | 11 | 1 | small cost change |
| Pose3 | 50 | 50 | 54 | 1 | maximum iterations |

Pose3 reached the default 50-iteration limit on both CPU and GPU. Its timing
is therefore a fair equal-configuration, equal-objective comparison at the
default iteration budget, but it should not be presented as a fully converged
solution.

## Detailed GPU timing breakdown

The table reports the median of each field independently across five complete
GPU runs, in milliseconds. These fields intentionally overlap and must not be
summed. In particular, `deviceInitializeWall` is inside
`persistentSetupWall`; pattern, structure, and setup-copy fields overlap
device initialization; the two `CpuSum` fields are sums across TBB workers and
overlap `factorLinearizationAndPackingWall`; and
`cudssDataInfoBoundaryWall` overlaps `cudssFactorAndSolve`.

| Stage, median milliseconds | Dubrovnik-16 | Dubrovnik-135 | Pose2 | Pose3 |
| --- | ---: | ---: | ---: | ---: |
| Internal profiled wall | 666.515 | 3,143.255 | 333.949 | 688.720 |
| Initial nonlinear error | 1.360 | 9.551 | 1.428 | 0.539 |
| Symbolic sparse plan | 48.573 | 300.306 | 33.155 | 7.223 |
| Persistent setup wall | 105.041 | 521.049 | 28.897 | 22.730 |
| Device initialization wall | 90.352 | 439.518 | 19.944 | 16.856 |
| Pattern H2D | 13.556 | 73.336 | 3.832 | 1.455 |
| Sparse structure setup | 22.695 | 106.485 | 7.927 | 9.606 |
| Setup D2H | 6.498 | 23.827 | 1.036 | 0.629 |
| Host numeric-buffer zeroing | 8.280 | 19.311 | 4.440 | 21.816 |
| Factor linearization + CSR pack wall | 56.783 | 145.707 | 53.093 | 97.797 |
| Factor linearization worker CPU sum | 202.650 | 751.644 | 311.312 | 559.413 |
| CSR packing worker CPU sum | 126.887 | 328.782 | 85.653 | 189.170 |
| Numeric H2D | 4.667 | 21.255 | 3.381 | 14.405 |
| Transpose-value update | 1.607 | 3.836 | 1.178 | 7.311 |
| Normal matrix `JᵀJ` | 1.693 | 6.518 | 0.602 | 4.772 |
| Normal RHS `Jᵀb` | 0.214 | 0.434 | 0.208 | 1.357 |
| Diagonal extraction | 0.042 | 0.068 | 0.043 | 0.349 |
| Old linear-model error | 0.068 | 0.066 | 0.081 | 0.646 |
| Damping preparation | 0.030 | 0.023 | 0.033 | 0.271 |
| Damping application | 0.060 | 0.150 | 0.115 | 0.358 |
| cuDSS analysis | 275.511 | 1,728.860 | 36.530 | 35.784 |
| cuDSS factorization + solve | 10.643 | 96.179 | 27.906 | 189.701 |
| cuDSS `DATA_INFO` host boundary | 8.169 | 95.692 | 23.002 | 164.090 |
| New linear-model error | 0.284 | 0.471 | 0.595 | 2.167 |
| Attempt D2H | 0.272 | 0.533 | 0.342 | 1.026 |
| Attempt host-record construction | 0.441 | 0.563 | 0.269 | 1.380 |
| CPU `Values::retract` | 41.152 | 96.481 | 46.859 | 96.965 |
| Nonlinear trial error | 7.941 | 28.713 | 13.213 | 25.117 |

## Transfer accounting

Pattern H2D and setup D2H are one-time setup traffic. Numeric H2D accumulates
over outer linearizations; attempt D2H accumulates over lambda attempts.
Counts were identical across the five runs for a workload.

| Workload | Pattern H2D | Numeric H2D | Total H2D | Setup D2H | Attempt D2H | Total D2H |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Dubrovnik-16 | 26.820 MiB | 83.033 MiB | 109.853 MiB | 36.526 MiB | 2.535 MiB | 39.061 MiB |
| Dubrovnik-135 | 174.102 MiB | 329.286 MiB | 503.389 MiB | 236.359 MiB | 6.252 MiB | 242.610 MiB |
| Pose2 | 10.140 MiB | 61.823 MiB | 71.963 MiB | 9.747 MiB | 2.518 MiB | 12.265 MiB |
| Pose3 | 5.351 MiB | 257.304 MiB | 262.655 MiB | 5.455 MiB | 5.439 MiB | 10.894 MiB |

Despite the transferred volume, copy time was not dominant. Median cumulative
numeric H2D was 3.7-26.0 ms, and attempt D2H was 0.25-1.33 ms.

## Interpretation

The SFM workloads remain dominated by one-time symbolic work. cuDSS analysis
was 41.3% of the internal wall time for Dubrovnik-16 and 55.0% for
Dubrovnik-135. Repeated factorization and solve were only 1.6% and 3.1%,
respectively. Reusing analysis across repeated same-topology solves remains
the clearest SFM optimization opportunity.

Pose2 is more balanced: factor linearization/packing wall was 15.9% of
internal time, CPU retract was 14.0%, cuDSS analysis was 10.9%, and repeated
factorization/solve was 8.4%. Here, improving state update and factor emission
would be more useful than focusing only on the sparse solve.

Pose3's 54 lambda attempts changed the profile. cuDSS factorization/solve was
27.5% of internal wall, factor linearization/packing was 14.2%, and CPU
retract was 14.1%. The overlapping mandatory cuDSS `DATA_INFO` boundary alone
accounted for 164.1 ms of the 189.7 ms factorization/solve interval, making
that host boundary a concrete target for long-attempt-count workloads.

The results support the generic CUDA path across SFM, Pose2, and Pose3 rather
than only BAL. They do not establish performance on other GPUs, and the
Pose3 result remains iteration-limited.
