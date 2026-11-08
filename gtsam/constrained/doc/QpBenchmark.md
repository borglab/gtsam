---
title: QP solver benchmarks
downloads:
  - file: qp-benchmark-report.html
    title: HTML report with interactive charts (internet connection required)
---

# QP solver benchmarks

This benchmark compares seven quadratic-programming solvers on robotic control and humanoid trajectory-optimization problems. The GTSAM entries are Dense Dual, Sparse Dual, and the direct Gaussian ALM solver with QR disabled. Measurements cover fixed QPs, rather than full nonlinear trajectory optimization.

Download the [interactive HTML report](qp-benchmark-report.html) or [per-case aggregates and provenance](qp-benchmark-summary.json). Interactive Plotly charts require an internet connection. Benchmark drivers, private inputs, raw calls and the separate ALM investigation stay in the ignored local benchmark workspace.

## Measurement protocol

The 64 public [MPC problems](https://qpsolvers.github.io/qpbenchmark/mpc_qpbenchmark/) run at Default, 1e-3, 1e-6 and 1e-9, giving 256 combinations per solver. Each has an initial call and five fresh repetitions. Timings use five-repeat medians. An original-QP certificate requires native success, finite primal feasibility, stationarity, absolute gap and multiplier-sign residuals below the setting’s threshold. Default uses native solver tolerances with an external threshold of 1.0; it is not an equal-accuracy comparison.

GTSAM timings include construction, setup, equality handling, initialization, solving, multiplier recovery and native output. Matrix-to-factor translation and scoring are excluded. External solvers use their matrix-input setup/solve boundaries. MPC runs are serial on CPU 0 with one TBB/numerical thread and a 10-second native watchdog. Profiles require all six calls to certify; failed cases remain in every denominator.

The direct ALM solver uses BCL/LM defaults with `params.lmParams.multifrontalParams.qrMode = gtsam.MultifrontalParameters.QRMode.Off`. This is a benchmark configuration; the public QR default remains Allow. Explicit accuracies change absolute stationarity and violation tolerances. See the [Gaussian solver guide](AugmentedLagrangianSolver.md) for the QP option call. ALM adds no objective regularization. Dense/Sparse Dual use proximal weight 1e-9 on MPC and zero added regularization on G1; DAQP retains its native selective proximal settings.

ALM QR-off measurements comprise 1,536 fresh calls on November 8. Dense/Sparse Dual and DAQP use the corresponding November 8 campaign; ProxQP, Clarabel and OSQP retain November 7 measurements on the same hardware and checksum-verified inputs. The HTML and JSON preserve source fingerprints and configuration details.

## MPC certificates

| Solver | Default | 1e-3 | 1e-6 | 1e-9 | Total / 256 |
| --- | --- | --- | --- | --- | --- |
| DAQP | 64 | 64 | 60 | 60 | 248 |
| GTSAM Dense Dual | 64 | 64 | 64 | 64 | 256 |
| GTSAM Sparse Dual | 64 | 64 | 64 | 64 | 256 |
| ProxQP | 64 | 46 | 63 | 62 | 235 |
| Clarabel | 64 | 64 | 64 | 64 | 256 |
| OSQP | 64 | 64 | 60 | 59 | 247 |
| GTSAM ALM (QR off) | 64 | 41 | 57 | 60 | 222 |

ProxQP passes 235/256 with multiplier signs checked, versus 253/256 under the published residual/gap checks alone. Native convergence by itself is not a certificate.

```{iframe} /qp-plots/mpc-success.html
:width: 100%
:class: qp-chart
:title: Original-QP certificates for all seven solvers

Original-QP certificates for all seven solvers
```

## MPC performance

Each profile shows the fraction of all cases solved within a time factor of the fastest certified solver. Curves farther left are faster; lower endpoints indicate failures. All 64 cases remain in each denominator.

```{iframe} /qp-plots/mpc-profile.html
:width: 100%
:class: qp-chart
:title: MPC profiles including QR-off Gaussian ALM and dual GTSAM solvers

MPC profiles including QR-off Gaussian ALM and dual GTSAM solvers
```

Default family times are medians of per-case five-repeat medians in milliseconds. Parentheses count cases that certify in all calls; successful subsets can differ.

| Family | DAQP | GTSAM Dense Dual | GTSAM Sparse Dual | ProxQP | Clarabel | OSQP | GTSAM ALM (QR off) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| LIPMWALK | 0.037 (30) | 0.095 (30) | 0.137 (30) | 0.135 (30) | 0.271 (30) | 0.351 (30) | 0.195 (30) |
| WHLIPBAL | 0.059 (30) | 0.192 (30) | 0.192 (30) | 0.302 (30) | 0.685 (30) | 0.477 (30) | 0.316 (30) |
| QUADCMPC | 7.992 (4) | 18.703 (4) | 8.378 (4) | 27.790 (4) | 3.035 (4) | 2.955 (4) | 9.817 (4) |

## QUADCMPC performance

The four larger MPC inputs retain original state/contact variable blocks. Hatched timing bars indicate at least one failed certificate.

```{iframe} /qp-plots/quad-profile.html
:width: 100%
:class: qp-chart
:title: QUADCMPC profiles with failures in every denominator

QUADCMPC profiles with failures in every denominator
```

```{iframe} /qp-plots/quad-times.html
:width: 100%
:class: qp-chart
:title: QUADCMPC timing medians; hatching marks failures

QUADCMPC timing medians; hatching marks failures
```

## G1 humanoid QPs

The three G1 captures use original Jacobians, noise weights and application damping, with no added objective regularization. Runs allow up to eight solver threads on CPUs 0–7, one numerical-library thread, 300 seconds and 16 GiB. OSQP retains its serial sparse backend. Original feasibility, stationarity, multiplier-sign and absolute-gap thresholds are 1e-7; complementarity is recorded separately.

The table reports primary complete-call seconds. Watchdogs are censored lower bounds. GTSAM factor reconstruction and external Hessian conversion are excluded. Retained G1SP20 successful external and Sparse Dual calls have five extra observations in the JSON; other G1 entries are single observations. DAQP was not run on G1.

| Capture | Solver | Seconds | Outcome | Gap |
| --- | --- | --- | --- | --- |
| G1SP20 | DAQP | — | not run | — |
| G1SP20 | GTSAM Dense Dual | — | memory_limit | — |
| G1SP20 | GTSAM Sparse Dual | 3.697 | certified | 1.68e-13 |
| G1SP20 | ProxQP | 1.002 | certified | 1.57e-08 |
| G1SP20 | Clarabel | 2.555 | certified | 2.83e-08 |
| G1SP20 | OSQP | 5.584 | certified | 7.41e-08 |
| G1SP20 | GTSAM ALM (QR off) | 42.154 | failed certificate | 2e-07 |
| G1CMCTWO | DAQP | — | not run | — |
| G1CMCTWO | GTSAM Dense Dual | — | memory_limit | — |
| G1CMCTWO | GTSAM Sparse Dual | 25.504 | certified | 8.54e-13 |
| G1CMCTWO | ProxQP | ≥300.001 | watchdog_timeout | — |
| G1CMCTWO | Clarabel | 14.489 | certified | 2.11e-08 |
| G1CMCTWO | OSQP | 22.504 | certified | 9.44e-08 |
| G1CMCTWO | GTSAM ALM (QR off) | 12.308 | certified | 1.88e-08 |
| G1SPFULL | DAQP | — | not run | — |
| G1SPFULL | GTSAM Dense Dual | — | memory_limit | — |
| G1SPFULL | GTSAM Sparse Dual | ≥300.001 | watchdog_timeout | — |
| G1SPFULL | ProxQP | ≥300.001 | watchdog_timeout | — |
| G1SPFULL | Clarabel | 42.293 | certified | 3.85e-09 |
| G1SPFULL | OSQP | 101.755 | certified | 9.77e-08 |
| G1SPFULL | GTSAM ALM (QR off) | ≥300.000 | watchdog_timeout | — |

Dense Dual reaches the memory cap on all captures. QR-off ALM certifies G1CMCTWO in 12.308 seconds. Its G1SP20 return takes 42.154 seconds but fails the absolute-gap threshold (about 2e-7 against 1e-7).

```{iframe} /qp-plots/g1-times.html
:width: 100%
:class: qp-chart
:title: G1 timings; hatched watchdog values are lower bounds

G1 timings; hatched watchdog values are lower bounds
```

## Datasets

| Dataset | Cases | Variables | Equalities | Inequality rows | Finite bound sides |
| --- | --- | --- | --- | --- | --- |
| LIPMWALK | 30 | 16 | 0 | 32 | 0 |
| WHLIPBAL | 30 | 50 | 0 | 100 | 0 |
| QUADCMPC1 | 1 | 768 | 384 | 512 | 512 |
| QUADCMPC2 | 1 | 576 | 288 | 384 | 384 |
| QUADCMPC3 | 1 | 384 | 192 | 256 | 128 |
| QUADCMPC4 | 1 | 384 | 192 | 256 | 256 |
| G1SP20 | 1 | 18549 | 0 | 230 | 0 |
| G1CMCTWO | 1 | 106836 | 0 | 378 | 0 |
| G1SPFULL | 1 | 193566 | 0 | 2437 | 0 |

MPC snapshot: `954906ac1f48b77e84a5bc314b7c2f00fb0d4ebc`. All public files and private G1 manifests were checksum-verified. Hardware: Intel Core i7-14700F; Release GCC 13.3.0/C++17 build with TBB. The HTML report lists solver versions, detailed configurations and provenance.
