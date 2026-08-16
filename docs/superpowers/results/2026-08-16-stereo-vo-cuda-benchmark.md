# Stereo Visual Odometry CUDA Benchmark Results

## Summary

The general hybrid CUDA LM optimizer supports GTSAM's
`GenericStereoFactor<Pose3, Point3>` without a factor-specific CUDA kernel.
Across three natural stereo VO sizes, every cuDSS and PCG run passed the
CPU/GPU endpoint gate. The best paired median speedup increased from **2.11x**
at 26 poses to **3.18x** at 77 poses and **3.25x** at 135 poses.

This measures the general full-normal hybrid optimizer: factor linearization,
`Values::retract`, and nonlinear error evaluation remain on the CPU, while the
normal-equation linear algebra and solve run on the GPU.

## Environment and protocol

- Revision: `e89fabb91f34521951ce223fb132d42ce41e3baa`, clean at benchmark time.
- Build: Release, CUDA 13.0, cuDSS enabled, TBB enabled, `sm_80`.
- GPU: NVIDIA A100 80 GB PCIe; driver `580.173.02`.
- CPU: 32-vCPU AMD EPYC-Milan host.
- Correct Between/Prior Jacobian build option: enabled.
- One unmeasured CPU/GPU warm-up pair per workload and configuration.
- Five measured pairs with CPU/GPU execution order alternated by repetition.
- Timed region: fresh optimizer construction plus `optimize()`; data loading
  and graph construction excluded.
- CPU: ordinary GTSAM LM using the point-first ordering.
- Direct rows: objective relative tolerance `1e-8`.
- PCG rows: relative residual tolerance `1e-6`, at most 5,000 iterations,
  block-Jacobi, warm start, and objective relative tolerance `1e-3`.
- CUDA fallback was disabled, so an unsupported factor would abort the run.

The three configurations were executed as separate processes. Each speedup
below therefore uses the CPU median paired with that configuration, avoiding
cross-process CPU timing substitution.

## Workloads

| Workload | Poses | Landmarks | Values | Stereo observations | Graph factors | Scalar columns | Residual rows |
|---|---:|---:|---:|---:|---:|---:|---:|
| stereo26 | 26 | 2,634 | 2,660 | 8,189 | 8,190 | 8,058 | 24,573 |
| stereo77 | 77 | 15,638 | 15,715 | 52,544 | 52,545 | 47,376 | 157,638 |
| stereo135 | 135 | 26,136 | 26,271 | 88,781 | 88,782 | 79,218 | 266,349 |

The extra graph factor is the first-pose prior. Each measurement has a
three-dimensional rectified stereo residual and connects one `Pose3` to one
`Point3`.

## End-to-end results

Times are five-run medians in seconds. `CPU/GPU` is the paired median speedup.

| Workload | CUDA configuration | CPU | GPU | CPU/GPU | Final objective | Maximum CPU/GPU difference | Iterations / attempts |
|---|---|---:|---:|---:|---:|---:|---:|
| stereo26 | cuDSS auto | 0.1882 | 0.0927 | **2.03x** | 1577.030618 | 5.00e-12 | 5 / 5 |
| stereo26 | cuDSS + GTSAM | 0.1919 | 0.0908 | **2.11x** | 1577.030618 | 7.28e-12 | 5 / 5 |
| stereo26 | PCG | 0.1953 | 0.1282 | **1.52x** | 1577.030618 | 7.00e-9 | 5 / 5 |
| stereo77 | cuDSS auto | 2.1561 | 0.6774 | **3.18x** | 7399.043517 | 5.46e-12 | 8 / 9 |
| stereo77 | cuDSS + GTSAM | 2.2145 | 0.6958 | **3.18x** | 7399.043517 | 9.09e-12 | 8 / 9 |
| stereo77 | PCG | 2.2587 | 0.9374 | **2.41x** | 7399.043517 | 2.32e-8 | 8 / 9 |
| stereo135 | cuDSS auto | 3.6493 | 1.1641 | **3.13x** | 12257.302157 | 1.82e-11 | 8 / 9 |
| stereo135 | cuDSS + GTSAM | 3.6683 | 1.1279 | **3.25x** | 12257.302157 | 5.46e-12 | 8 / 9 |
| stereo135 | PCG | 3.8938 | 1.7203 | **2.26x** | 12257.302157 | 4.23e-8 | 8 / 9 |

All direct runs analyzed the fixed sparse structure exactly once. All PCG runs
completed without a cap hit or breakdown and reported a converged final solve.

## Ordering result

The machine artifacts verify `user_ordering_applied=true` for every
cuDSS-with-GTSAM run and `false` for cuDSS-auto and PCG. The supplied ordering
places every `Point3` before every `Pose3` and is expanded to a scalar
permutation while keeping each variable block contiguous.

The end-to-end GPU difference relative to cuDSS-auto was small and not
monotonic: GTSAM ordering was 2.0% faster on stereo26, 2.7% slower on stereo77,
and 3.1% faster on stereo135. The direct-backend cumulative medians do not show
a corresponding large factorization improvement:

| Workload | Ordering | Analysis | Factorization | Solve |
|---|---|---:|---:|---:|
| stereo26 | auto | 0.0300 | 0.00183 | 0.00068 |
| stereo26 | GTSAM | 0.0298 | 0.00222 | 0.00070 |
| stereo77 | auto | 0.1129 | 0.00772 | 0.00260 |
| stereo77 | GTSAM | 0.1151 | 0.01291 | 0.00312 |
| stereo135 | auto | 0.1870 | 0.01198 | 0.00320 |
| stereo135 | GTSAM | 0.1953 | 0.02174 | 0.00522 |

Consequently, this campaign does not support claiming that the GTSAM
point-first permutation materially improves stereo-VO factorization. The small
wall-time differences include normal run-to-run variation in the hybrid CPU
frontend. cuDSS automatic ordering remains the simpler default for this
problem family.

## PCG behavior

PCG used 1,430 total iterations over five solves on stereo26, 4,580 over nine
solves on stereo77, and 6,950 over nine solves on stereo135. Median cumulative
PCG solve times were 0.0758, 0.3736, and 0.7160 seconds, respectively. PCG was
faster than CPU on every size but slower than cuDSS because the iteration count
grew with the problem and the block-Jacobi preconditioner did not exploit an
explicit point-elimination structure.

## Interpretation and caveats

1. This is a genuine different factor family from BAL and pose graphs, but the
   measured optimizer is still hybrid. It validates the general Jacobian
   frontend rather than a CUDA implementation of `GenericStereoFactor`.
2. The upstream example anchors the first pose with
   `NonlinearEquality<Pose3>`, which produces a constrained linear factor and
   is unsupported by the general CUDA sparse-Jacobian frontend. This benchmark
   replaces it with a finite unit-noise `PriorFactor<Pose3>` for both CPU and
   GPU. Results are internally fair but are not numerically identical to the
   original equality-constrained example.
3. The point-first ordering guides sparse factorization of the complete
   `J^T J`; the general optimizer does not explicitly construct and solve a
   camera-only Schur complement.
4. GPU stage timers overlap and must not be summed into an exclusive total.
5. PCG is validated with a deliberately looser endpoint gate than the direct
   solver and should not be presented as bit-level direct-solver parity.

## Artifacts

Raw JSON and CSV files are in
`timing/cuda_sparse/benchmark_logs/20260816_stereo_vo/`:

- `results.json.cudss-auto.json` / `results.csv.cudss-auto.csv`
- `results.json.cudss-gtsam.json` / `results.csv.cudss-gtsam.csv`
- `results.json.pcg.json` / `results.csv.pcg.csv`

The JSON contains every raw repetition, system and transfer sizes, solver
statistics, LM attempt traces, detailed stage timings, hardware metadata, and
the exact clean benchmark revision.
