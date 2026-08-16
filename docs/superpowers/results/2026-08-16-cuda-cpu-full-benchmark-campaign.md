# CUDA/CPU full benchmark campaign

> This report records the historical `1e-10` PCG benchmark profile. The
> current named profile and its rerun are documented in
> [PCG benchmark profile at `1e-6`](2026-08-16-pcg-1e-6-benchmark.md).

Date: 2026-08-16

Revision: `1d1d6c5f0dae` (`codex/cuda-hybrid-final`, clean before results)

Machine: NVIDIA A100 80 GB PCIe, CUDA 13.0, driver 580.173.02, Release,
`sm_80`, TBB enabled, correct Between/Prior Jacobians enabled.

## Method

The primary general-solver matrix used `timeCudaSparseLM` with one untimed
CPU/GPU warmup and three measured pairs. Even repetitions ran CPU then GPU;
odd repetitions reversed the order. Reported times are medians and include
optimizer construction plus optimization, but exclude dataset loading and
initialization. CPU fallback was disabled. Direct rows required CPU/GPU final
objective agreement at relative tolerance `1e-8`. PCG used block Jacobi,
warm starts, relative residual `1e-10`, at most 5,000 iterations, and a stated
objective tolerance of `1e-3`; no general PCG solve hit its cap or broke down.

The CPU baseline is ordinary GTSAM LM with multifrontal Cholesky. The general
GPU path remains hybrid: factor linearization/CSR packing, `Values::retract`,
and nonlinear error evaluation are CPU work; numerical linear solving is CUDA.

## General solver: CPU versus three GPU configurations

| Problem | GPU configuration | CPU median (s) | GPU median (s) | Speedup | Final objective | Termination |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| BAL16 | cuDSS auto | 2.144 | 0.642 | 3.34x | 18034.1030543 | converged |
| BAL16 | cuDSS + GTSAM ordering | 2.393 | 0.614 | 3.90x | 18034.1030543 | converged |
| BAL16 | PCG | 2.162 | 0.403 | **5.36x** | 18034.1030543 | converged |
| BAL135 | cuDSS auto | 6.333 | 3.187 | 1.99x | 378041.329924 | small cost change |
| BAL135 | cuDSS + GTSAM ordering | 6.275 | 2.833 | 2.22x | 378041.329924 | small cost change |
| BAL135 | PCG | 7.642 | 1.757 | **4.35x** | 378041.329924 | small cost change |
| Pose2 w10000 | cuDSS auto | 4.298 | 1.091 | **3.94x** | 162.033721791 | converged |
| Pose2 w10000 | cuDSS + GTSAM ordering | 4.271 | 1.506 | 2.83x | 162.033721791 | converged |
| Pose2 w10000 | PCG | 4.547 | 3.590 | 1.27x | 162.033721791 | converged |
| Pose3 sphere | cuDSS auto | 3.224 | 0.757 | **4.26x** | 5232.19973886 | 50-iteration limit |
| Pose3 sphere | cuDSS + GTSAM ordering | 3.278 | 1.203 | 2.72x | 5232.19973886 | 50-iteration limit |
| Pose3 sphere | PCG | 3.605 | 1.132 | 3.18x | 5232.19973886 | 50-iteration limit |

All twelve rows passed their configured objective gate. Pose3 is a fair
same-budget/same-objective comparison, but not evidence of convergence: CPU and
GPU both consumed the 50-iteration budget. General PCG required 1,050, 1,940,
33,960, and 9,040 accumulated inner iterations for BAL16, BAL135, Pose2, and
Pose3 respectively, without a cap hit.

Ordering is workload dependent. GTSAM ordering improved direct GPU time by 4%
on BAL16 and 11% on BAL135, but was 38% slower on Pose2 and 59% slower on
Pose3 than cuDSS automatic ordering. PCG was the best general backend for both
BAL problems, while cuDSS auto was decisively better for Pose2 and Pose3.

## BAL: ordinary CPU, specialized CUDA SFM, and general CUDA

The existing three-way harness was repeated three times per dataset; each
invocation performed its own untimed warmups. It uses the same graph, initial
values, LM parameters, and objective for all three implementations.

| BAL problem | Cameras / points / observations | CPU median (s) | Specialized CUDA SFM (s) | Specialized speedup | General CUDA (s) | General speedup |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| BAL16 | 16 / 22,106 / 83,718 | 2.406 | 0.0916 | **26.3x** | 0.657 | 3.66x |
| BAL88 | 88 / 64,298 / 383,937 | 6.432 | 0.262 | **24.5x** | 2.189 | 2.94x |
| BAL135 | 135 / 90,642 / 553,336 | 7.256 | 0.445 | **16.3x** | 3.118 | 2.33x |

Objectives agreed within the harness tolerance in all nine measured
comparisons. This is the clearest result for Frank's desired architecture:
the shared general solver provides a real 2.3x–3.7x end-to-end gain without
requiring CUDA implementations for every factor, while the fully specialized
SFM producer is 16x–26x faster because projection linearization, Schur
construction, point recovery, and retraction are specialized as well.

## Specialized SFM formulation/backend matrix

These timings exclude downloading final `Values`. BAL16 passed the seven-row
objective/convergence gate:

| Configuration | BAL16 wall (s) | Status |
| --- | ---: | --- |
| Schur dense | 0.026 | pass, fastest |
| Schur cuDSS auto | 0.135 | pass |
| Schur cuDSS GTSAM | 0.137 | pass |
| Schur PCG | 0.532 | pass |
| Full normal cuDSS auto | 0.324 | pass |
| Full normal cuDSS GTSAM | 0.312 | pass |
| Full normal PCG | 0.723 | pass |

BAL135 demonstrates that a small reduced-camera system still strongly favors
dense Schur:

| Configuration | BAL135 wall (s) | Status |
| --- | ---: | --- |
| Schur dense | **0.155** | objective reference |
| Schur cuDSS auto | 4.992 | objective pass |
| Schur cuDSS GTSAM | 4.920 | objective pass |
| Schur PCG | 8.653 | failed: one inner cap hit |
| Full normal cuDSS auto | 2.300 | objective pass |
| Full normal cuDSS GTSAM | 1.774 | objective pass |
| Full normal PCG | 12.185 | failed: five inner cap hits |

BAL88 is useful as a diagnostic problem, but its specialized solver matrix did
not pass the strict `1e-8` endpoint gate. Repeated identical Schur-cuDSS runs
finished between 298,807 and 298,817; repeated dense runs varied between
298,811.95 and 298,812.23. The persistent block and reduced Schur kernels use
floating-point `atomicAdd`, so accumulation order is scheduling dependent and
the nonlinear trajectory amplifies the numerical perturbation. The measured
wall times remain useful, but these rows must not be called correctness-gated:

| Configuration | BAL88 wall (s) | Strict status |
| --- | ---: | --- |
| Schur dense | 0.106 | reference; small run-to-run endpoint drift |
| Schur cuDSS auto | 3.007 | endpoint gate failed |
| Schur cuDSS GTSAM | 3.053 | endpoint gate failed |
| Schur PCG | 5.782 | endpoint gate and one cap hit |
| Full normal cuDSS auto | 1.417 | endpoint gate failed |
| Full normal cuDSS GTSAM | 1.221 | endpoint gate failed |
| Full normal PCG | 7.814 | endpoint gate and three cap hits |

Do not merely loosen the BAL88 threshold. First decide whether the release
contract requires deterministic accumulation, solution-vector parity, or a
trajectory-aware final-objective tolerance.

## Pose2 initialization and LM semantics

Initialization cost is recorded separately and excluded from CPU/GPU solver
wall time.

| Setup | Initialization (s) | CPU (s) | GPU (s) | Speedup | GPU iterations / attempts | Objective |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Default LM, raw, cuDSS auto | 0 | 4.298 | 1.091 | 3.94x | 32 / 34 | 162.0337 |
| Default LM, FastSync, cuDSS auto | 0.676 | 6.055 | 1.482 | 4.09x | 43 / 44 | 175.7621 |
| Upstream LM/prior, raw, GTSAM order | 0 | 1.049 | 0.531 | 1.98x | 9 / 9 | 144.8629 |
| Upstream LM/prior, FastSync, GTSAM order | 0.625 | 1.792 | 0.758 | 2.36x | 10 / 17 | 144.9100 |

FastSync is not automatically faster. With the current correct Jacobians it
increased work in both parameter regimes, and including its 0.62–0.68 s setup
cost makes it less attractive still. The two objectives across default versus
upstream settings are not directly comparable because the anchor noise and LM
defaults differ.

## Additional large workloads

### Pose2 w20000

`w20000.txt` contains 20,061 poses and 26,832 factors after anchoring. It was
run through the paired harness by mapping it to the harness's Pose2 input slot;
all rows converged and passed objective gates.

| GPU configuration | CPU (s) | GPU (s) | Speedup |
| --- | ---: | ---: | ---: |
| cuDSS auto | 5.813 | 1.396 | **4.16x** |
| cuDSS + GTSAM ordering | 5.880 | 3.033 | 1.94x |
| PCG | 5.592 | 1.608 | 3.48x |

This confirms the Pose2 result on a second, larger state dimension and a much
sparser graph. `T1_city10000_04.txt` was also attempted, but `load2D` rejected
its covariance encoding, so no CPU/GPU number is reported.

### Candidates requiring a paired harness

- A 1,000-step synthetic IMU chain has roughly 15,015 scalar unknowns and
  15,165 residual rows. Its current executable compares CPU linearization
  representations, not CPU versus CUDA LM.
- Victoria Park has 7,120 values and 10,609 heterogeneous Pose2/Point2
  factors. Its current benchmark is incremental ISAM2, so comparing it with
  batch CUDA LM would be invalid.
- The large stereo-VO files contain about 26,271 values and 88,782 factors,
  but the current graph uses a constrained nonlinear equality and stereo
  factors that are not accepted by the general CUDA streaming path.

These are worthwhile next-domain tests only after adding a paired batch-LM
harness with identical graphs, anchors, initialization, and stopping criteria.

## Reproduction and raw data

Primary command:

```bash
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --all-cuda-configurations \
  --datasets bal16,bal135,pose2,pose3 \
  --data-dir /tmp/cu-gtsam-benchmark-campaign-data \
  --warmups 1 --repeats 3 \
  --json timing/cuda_sparse/benchmark_logs/20260816_cpu_gpu_full_campaign/results.json \
  --csv timing/cuda_sparse/benchmark_logs/20260816_cpu_gpu_full_campaign/results.csv
```

Raw records are under:

- `timing/cuda_sparse/benchmark_logs/20260816_cpu_gpu_full_campaign/`
- `timing/cuda_sparse/benchmark_logs/20260816_pose2_initialization_matrix/`
- `timing/cuda_sparse/benchmark_logs/20260816_additional_pose_graphs/`
- `timing/sfm_ba/benchmark_logs/20260816_cpu_specialized_general/`
- `timing/sfm_ba/benchmark_logs/20260816_bal135_persistent_solver_matrix/`
- `timing/sfm_ba/benchmark_logs/20260816_bal88_persistent_solver_matrix/`
