# PCG benchmark profile at `1e-6`

Date: 2026-08-16

Base revision: `5d2f6b0e4488` (`codex/cuda-hybrid-final`)

Machine: NVIDIA A100 80 GB PCIe, CUDA 13.0, driver 580.173.02, Release,
`sm_80`, TBB enabled, correct Between/Prior Jacobians enabled.

## Change

The named PCG benchmark configurations now use relative residual tolerance
`1e-6` instead of `1e-10`. The general benchmark retains its 5,000-iteration
cap, block-Jacobi preconditioner, warm start, and 0.1% final-objective gate.
The specialized SFM benchmark retains its 1,000-iteration-per-solve cap. The
public `CudaPcgOptions` default was already `1e-6` and was not changed.

## General solver benchmark

The general benchmark used one untimed CPU/GPU warmup and three measured
pairs. GPU wall time and accumulated inner iterations are medians. The old
rows are the immediately preceding `1e-10` campaign on the same machine.

| Problem | GPU time at `1e-10` (s) | GPU time at `1e-6` (s) | Inner iterations at `1e-10` | Inner iterations at `1e-6` | New CPU/GPU speedup |
| --- | ---: | ---: | ---: | ---: | ---: |
| BAL16 | 0.403 | 0.413 | 1,050 | 920 | 6.10x |
| BAL135 | 1.757 | 1.542 | 1,940 | 1,490 | 4.73x |
| Pose2 w10000 | 3.590 | 2.313 | 33,960 | 17,440 | 1.99x |
| Pose3 sphere | 1.132 | 0.862 | 9,050 | 5,290 | 4.18x |

All four new rows passed the 0.1% CPU/GPU final-objective gate. No solve hit
the iteration cap or broke down, and the last solve converged. Pose3 still
terminated at the shared 50-iteration outer limit, so it remains a
same-budget comparison rather than convergence-to-solution evidence. BAL16's
small wall-time regression despite fewer inner iterations is within the noise
visible in the paired CPU/GPU run; the larger workloads improved materially.

## Specialized SFM PCG diagnostic

These are single warmed runs of each affected configuration, so they are
diagnostic rather than robust timing statistics.

| Problem / formulation | Wall at `1e-10` (s) | Wall at `1e-6` (s) | Inner iterations, old to new | Cap hits, old to new | New final objective |
| --- | ---: | ---: | ---: | ---: | ---: |
| BAL16 Schur | 0.532 | 0.277 | 1,090 to 520 | 0 to 0 | 18033.9173 |
| BAL16 full normal | 0.723 | 0.183 | 2,130 to 470 | 0 to 0 | 18025.2506 |
| BAL135 Schur | 8.653 | 2.634 | 7,920 to 2,220 | 1 to 0 | 373823.0154 |
| BAL135 full normal | 12.185 | 2.658 | 10,120 to 2,140 | 5 to 0 | 387659.1269 |
| BAL88 Schur | 5.782 | 1.923 | 6,390 to 1,910 | 1 to 0 | 293503.5139 |
| BAL88 full normal | 7.814 | 3.373 | 9,180 to 3,900 | 3 to 0 | 293804.7113 |

All six solves now report convergence without cap hits or breakdowns. The
looser inner tolerance can change the nonlinear LM trajectory, however. In
particular, BAL135 full normal ends about 3.7% above the old PCG endpoint;
the BAL88 endpoints also move materially (in this run, downward). Therefore
these specialized rows support the speed and cap-hit effect of `1e-6`, but do
not establish strict cross-formulation endpoint parity.

## Raw data

- `timing/cuda_sparse/benchmark_logs/20260816_pcg_1e-6_campaign/`
- `timing/sfm_ba/benchmark_logs/20260816_sfm_pcg_1e-6/`
