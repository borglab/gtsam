# CUDA final extended-dataset benchmark

Date: 2026-08-16

Revision: `c949b0736afa59a6b884be9242980488d74ebbb6`
(`codex/cuda-hybrid-final`, clean in every raw record)

## Executive summary

The final general CUDA LM implementation was benchmarked on 16 workloads and
three GPU configurations, producing 48 paired CPU/GPU comparisons. The
campaign covers four BAL sizes, seven Pose2/Point2 workloads, and five Pose3
sphere sizes. Ten inputs are original datasets and six are connected induced
prefixes used only for controlled size scaling.

The strongest release result is cuDSS automatic ordering. It beat ordinary
CPU GTSAM LM on 14 of 16 workloads, reached 2.00x--4.20x on every medium or
large direct-solver workload, and reached 4.02x on the heterogeneous Victoria
Park graph. It lost only on the two tiny setup-dominated problems: BAL3 and
Pose2 w100. The 119-value heterogeneous example was approximately break-even
at 1.08x.

GTSAM ordering is decisively workload dependent. Point-first Schur ordering
improved BAL GPU wall time by 4.7% on BAL16, 8.5% on BAL88, and 13.3% on
BAL135. GTSAM COLAMD was slower than cuDSS automatic ordering on every Pose2,
Pose3, and heterogeneous workload, by 12%--121%.

PCG was the fastest GPU backend for BAL16/88/135, reaching 6.07x, 4.92x, and
4.61x CPU speedup. It was also competitive on sparse Pose2 w20000 and
Victoria Park, but was poor on several smaller or denser pose graphs because
it needed tens of thousands of accumulated inner iterations. These PCG rows
use a looser `1e-3` endpoint gate and therefore must not be presented as the
same strict direct-solver parity claim.

## Machine and build

- GPU: NVIDIA A100 80 GB PCIe, compute capability 8.0, ECC enabled
- Driver/runtime/compiler: driver 580.173.02, CUDA runtime 13.0, nvcc 13.0.88
- cuDSS: 0.8.0.10
- CPU: 32-vCPU AMD EPYC-Milan virtual machine, one NUMA node
- Host compiler: GCC 13.3.0; TBB 2021.11
- OS: Ubuntu kernel `6.8.0-124-generic`
- Build: Release, `sm_80`, TBB enabled, correct Between/Prior Jacobians enabled
- Benchmark executable SHA-256:
  `3667c9b699d8459e74931484c93e97e5e00abb591d8953ffb14c1fe7e94d8e43`

No concurrent CUDA compute process was present when the campaign began. The
GPU runs were serialized.

## Method

The paired `timeCudaSparseLM` harness was used without solver-code changes.
Each sample includes fresh optimizer construction and `optimize()`, but
excludes dataset loading and initialization. CPU and GPU use the same graph,
values, noise models, prior, LM settings, iteration budget, and stopping
criteria. Measured pairs alternate CPU-then-GPU and GPU-then-CPU.

The four core workloads use one untimed warmup pair and five measured pairs.
BAL88, Pose2 w5000/w20000, Victoria Park, and Pose3 sphere1000/sphere1500 also
use one plus five. Pose2 w1000 and Pose3 sphere500 use one plus nine. The four
shortest cases use two warmups plus 15 measurements: BAL3, Pose2 w100, the
119-value heterogeneous example, and Pose3 sphere100.

Configurations:

- `cudss-auto`: cuDSS direct SPD solve with backend-native ordering.
- `cudss-gtsam`: cuDSS direct SPD solve with a supplied GTSAM permutation.
  BAL receives point-first Schur ordering; other graphs receive COLAMD.
- `pcg`: matrix-free PCG, relative residual `1e-6`, maximum 5,000 iterations,
  block-Jacobi preconditioning, and warm starts.

Direct rows require final-objective agreement at scaled relative tolerance
`1e-8`. PCG requires `1e-3`, no cap hit, no breakdown, and a converged final
linear solve. The harness also gates finite output, actual CUDA execution, and
CPU repeatability.

Bootstrap intervals quoted below use 20,000 deterministic resamples of the
paired CPU/GPU ratios and report the 2.5th and 97.5th percentiles of the median.
With only five samples these intervals are descriptive, not asymptotic
confidence guarantees.

## Complete results

The CPU column is the median from the `cudss-auto` paired campaign. Each
reported speedup is the harness's median of paired CPU/GPU ratios from its own
configuration, so the speedup does not necessarily equal that displayed CPU
column divided by the GPU median.

| Workload | Values | Factors | CPU (s) | cuDSS auto (s / speedup) | GTSAM order (s / speedup) | PCG (s / speedup) | Termination |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| BAL3 | 10 | 18 | 0.0065 | 0.0234 / 0.28x | 0.0228 / 0.27x | 0.1065 / 0.06x | 20-iteration limit |
| BAL16 | 22,122 | 83,718 | 2.3883 | 0.6607 / 3.61x | 0.6296 / 4.11x | **0.4022 / 6.07x** | converged |
| BAL88 | 64,386 | 383,937 | 6.4096 | 2.3770 / 2.70x | 2.1751 / 2.98x | **1.3343 / 4.92x** | converged |
| BAL135 | 90,777 | 553,336 | 6.4687 | 3.2381 / 2.00x | 2.8059 / 2.51x | **1.4691 / 4.61x** | small cost change |
| Pose2 w100 | 100 | 301 | 0.0137 | **0.0196 / 0.70x** | 0.0220 / 0.59x | 0.0480 / 0.28x | converged |
| Pose2 w1000 prefix | 1,000 | 3,748 | 0.3695 | **0.1066 / 3.47x** | 0.1681 / 2.25x | 1.9108 / 0.21x | converged |
| Pose2 w5000 prefix | 5,000 | 30,069 | 0.9560 | **0.3078 / 3.11x** | 0.3963 / 2.45x | 2.5075 / 0.38x | converged |
| Pose2 w10000 | 10,000 | 64,312 | 4.5629 | **1.1555 / 3.95x** | 1.5846 / 2.76x | 2.2146 / 1.89x | converged |
| Pose2 w20000 | 20,061 | 26,832 | 5.3818 | **1.2818 / 4.20x** | 2.8338 / 1.85x | 1.4683 / 3.71x | converged |
| Mixed Pose2/Point2 example | 119 | 517 | 0.0242 | **0.0224 / 1.08x** | 0.0272 / 0.89x | 0.0398 / 0.65x | converged |
| Victoria Park | 7,120 | 10,609 | 3.8800 | **0.9650 / 4.02x** | 1.4743 / 2.55x | 0.9705 / 3.87x | 50-iteration limit |
| Pose3 sphere100 prefix | 100 | 249 | 0.1528 | **0.0683 / 2.24x** | 0.1320 / 0.93x | 1.0058 / 0.14x | converged |
| Pose3 sphere500 prefix | 500 | 1,849 | 0.6528 | **0.2274 / 2.87x** | 0.3115 / 2.17x | 2.0230 / 0.34x | 50-iteration limit |
| Pose3 sphere1000 prefix | 1,000 | 3,849 | 1.0061 | **0.3004 / 3.35x** | 0.4241 / 2.32x | 0.9715 / 1.05x | converged |
| Pose3 sphere1500 prefix | 1,500 | 5,849 | 2.2312 | **0.5563 / 4.01x** | 0.7524 / 3.07x | 0.6335 / 3.61x | 50-iteration limit |
| Pose3 sphere2200 | 2,200 | 8,648 | 3.4543 | **0.8243 / 4.19x** | 1.2948 / 2.68x | 0.8463 / 3.90x | 50-iteration limit |

All 48 rows passed their configured endpoint gate. The worst direct relative
objective difference was `5.68e-13`, far below `1e-8`. The worst PCG relative
difference was `1.23e-4`, below `1e-3`. Every PCG run finished its last linear
solve converged, with zero iteration-cap hits and zero breakdowns.

BAL3, Victoria Park, and the 500/1500/2200-pose Pose3 graphs reached their
shared CPU/GPU outer-iteration limit. Their timing and objective parity are
valid same-budget comparisons, but they are not evidence of convergence to a
solution.

## cuDSS-auto scaling

| Workload | CPU/GPU speedup | Bootstrap 95% interval |
| --- | ---: | ---: |
| BAL3 | 0.28x | [0.27, 0.29] |
| BAL16 | 3.61x | [3.49, 3.67] |
| BAL88 | 2.70x | [2.52, 2.91] |
| BAL135 | 2.00x | [1.98, 2.02] |
| Pose2 w100 | 0.70x | [0.57, 0.70] |
| Pose2 w1000 prefix | 3.47x | [2.92, 3.89] |
| Pose2 w5000 prefix | 3.11x | [2.98, 3.23] |
| Pose2 w10000 | 3.95x | [3.85, 4.11] |
| Pose2 w20000 | 4.20x | [4.05, 4.38] |
| Mixed Pose2/Point2 example | 1.08x | [1.05, 1.10] |
| Victoria Park | 4.02x | [3.95, 4.24] |
| Pose3 sphere100 prefix | 2.24x | [2.13, 2.26] |
| Pose3 sphere500 prefix | 2.87x | [2.83, 3.18] |
| Pose3 sphere1000 prefix | 3.35x | [3.11, 3.57] |
| Pose3 sphere1500 prefix | 4.01x | [3.99, 4.16] |
| Pose3 sphere2200 | 4.19x | [3.88, 4.41] |

The Pose2 crossover lies between 100 and 1,000 poses for this graph family and
machine. Pose3 crosses earlier in this campaign because the ordinary CPU
Pose3 factors and multifrontal solves are more expensive per variable; even
the 100-pose induced graph reaches 2.24x. BAL does not provide a dense enough
small-size ladder to locate its crossover: BAL3 is setup dominated, while the
next available problem is BAL16 with 22,122 values.

BAL speedup decreases from BAL16 to BAL135 for cuDSS direct solving because
symbolic analysis of the full normal equations grows disproportionately. This
is consistent with the separate specialized Schur solver result: the general
architecture is portable across factors, but full-normal sparse analysis is
not the best BA-specific formulation.

## Ordering result

Relative GPU-wall change from replacing cuDSS automatic ordering with GTSAM
ordering:

- BAL16: -4.7%; BAL88: -8.5%; BAL135: -13.3%.
- Pose2 original/controlled workloads: +12% to +121%.
- Mixed example and Victoria Park: +22% and +53%.
- Pose3 workloads: +35% to +93%.

Thus there is no contradiction between the BAL ordering win and the pose-graph
loss. BAL uses a problem-specific point-before-camera elimination structure.
For the other workloads, supplying block-level COLAMD disables cuDSS's own
reordering and produces a worse factor structure than backend-native ordering.
The production default should remain automatic, while BAL-aware callers may
select the supplied Schur ordering.

## PCG sensitivity

Median accumulated PCG iterations varied substantially:

- BAL16/88/135: 920 / 1,860 / 1,430.
- Pose2 w1000/w5000/w10000: 39,940 / 43,860 / 17,440.
- Sparse Pose2 w20000 and Victoria Park: 4,740 and 2,670.
- Pose3 sphere100/500/1000/1500/2200: 20,010 / 38,080 / 15,170 /
  5,780 / 5,290.

The result is topological rather than a simple function of scalar dimension.
PCG is excellent for the BAL block structure and competitive on the larger,
sparser pose graphs, but it is not a robust universal default under the current
block-Jacobi preconditioner.

## Dataset provenance

Original datasets:

- BAL Dubrovnik 3/16/88/135.
- Pose2 w100, w10000, and w20000.
- `example.graph` and Victoria Park, both containing Pose2 odometry and
  Pose2/Point2 bearing-range factors.
- Pose3 `sphere_smallnoise.graph` at 2,200 poses.

Controlled scaling inputs:

- Pose2 w1000 and w5000 contain `VERTEX2` keys below the cutoff and every
  `EDGE2` whose two endpoints are below the cutoff, taken from w10000.
- Pose3 sphere100/500/1000/1500 use the analogous induced-prefix rule on
  `VERTEX3`/`EDGE3` from the 2,200-pose sphere.

All six induced graphs were checked to have exactly one connected component.
They must be described as controlled subgraphs of a parent dataset rather than
as independent real-world benchmark datasets. Exact byte sizes, SHA-256s,
dimensions, nonzero counts, and source paths are in `dataset_manifest.csv`.

The Pose3 harness intentionally assigns unit six-dimensional noise to accepted
legacy `EDGE3` measurements because `sphere_smallnoise.graph` does not carry
information matrices. CPU and GPU receive the same graph, but the objective is
not comparable to a weighted g2o Pose3 benchmark.

## Screened but excluded

The local workspace was also screened for other candidate problems:

- `T1_city10000_04.txt`: rejected by `load2D` because its covariance encoding
  is not one of the supported formats.
- `sphere2500.txt`: edge-only and has no initial vertices; its information
  matrices are also rejected by the benchmark's intentionally strict legacy
  parser. Inventing an initialization would define a different benchmark.
- Small quaternion g2o Pose3 files: the current paired harness accepts their
  vertices but not `EDGE_SE3:QUAT` edges.
- Stereo VO, IMU, Plaza, and Bundler inputs: locally available, but require
  domain-specific graph construction or format conversion. Adding them would
  require a new paired harness and an explicit benchmark definition, rather
  than merely exercising the unchanged final code.
- Several 4--11-pose and 2-camera examples were omitted as redundant
  correctness smokes after BAL3, Pose2 w100, and the mixed 119-value graph
  already established the setup-dominated regime.

## Reproduction

Core command:

```bash
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --all-cuda-configurations \
  --datasets bal16,bal135,pose2,pose3 \
  --data-dir /home/ubuntu/cu-gtsam/examples/Data \
  --warmups 1 --repeats 5 \
  --json timing/cuda_sparse/benchmark_logs/20260816_extended_size_campaign/core/results.json \
  --csv timing/cuda_sparse/benchmark_logs/20260816_extended_size_campaign/core/results.csv
```

The remaining inputs were staged under the fixed filenames required by the
unchanged harness and run separately. For example:

```bash
mkdir -p /tmp/cuda-final-expanded-size-data/pose2-w1000
awk '$1=="VERTEX2" && $2<1000 {print} \
     $1=="EDGE2" && $2<1000 && $3<1000 {print}' \
  /home/ubuntu/cu-gtsam/examples/Data/w10000.graph \
  > /tmp/cuda-final-expanded-size-data/pose2-w1000/w10000.graph

./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --all-cuda-configurations --datasets pose2 \
  --data-dir /tmp/cuda-final-expanded-size-data/pose2-w1000 \
  --warmups 1 --repeats 9 \
  --json timing/cuda_sparse/benchmark_logs/20260816_extended_size_campaign/pose2-w1000/results.json \
  --csv timing/cuda_sparse/benchmark_logs/20260816_extended_size_campaign/pose2-w1000/results.csv
```

Raw data and derived tables:

- `timing/cuda_sparse/benchmark_logs/20260816_extended_size_campaign/`
- `summary.csv`: all 48 logical rows, solver statistics, variability, and
  bootstrap intervals.
- `dataset_manifest.csv`: provenance, hashes, dimensions, and sparsity.

GPU stage timings overlap and must not be stacked or summed as exclusive wall
time.
