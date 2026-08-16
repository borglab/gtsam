# Stereo Visual Odometry CUDA Benchmark Design

## Objective

Extend the paired CPU/CUDA Levenberg-Marquardt benchmark to cover GTSAM's
three local stereo visual-odometry workloads. Produce report-ready CPU,
cuDSS-auto, cuDSS-with-GTSAM-ordering, and PCG measurements for every size.

## Workloads

The campaign uses the existing calibration, camera-pose, and stereo-factor
files without truncating or synthesizing observations:

| Workload | Poses | Landmarks | Stereo factors |
|---|---:|---:|---:|
| `stereo26` | 26 | 2,634 | 8,189 |
| `stereo77` | 77 | 15,638 | 52,544 |
| `stereo135` | 135 | 26,136 | 88,781 |

Each graph contains `GenericStereoFactor<Pose3, Point3>` factors with the
calibration and unit three-dimensional measurement model used by
`StereoVOExample_large`. Camera poses come from the supplied pose file. The
first observation of each landmark supplies its camera-frame triangulation,
which is transformed through the corresponding initial camera pose to form
the initial world point.

The example's `NonlinearEquality<Pose3>` anchor is not compatible with the
general CUDA sparse-Jacobian frontend because it produces a constrained
linear factor. The benchmark instead adds a finite unit-noise
`PriorFactor<Pose3>` on the first pose. The same graph, values, noise models,
and LM parameters are passed to the CPU and GPU optimizers.

## Ordering

The CPU baseline uses a point-first ordering followed by camera poses, which
preserves the normal bundle-adjustment elimination structure. The
`cudss-gtsam` row expands that same key ordering to scalar columns and supplies
it to cuDSS. The `cudss-auto` row lets cuDSS choose its own permutation. PCG is
ordering-independent.

This ordering guides factorization of the full normal matrix; it does not turn
the general optimizer into an explicit Schur-complement solver.

## Measurement Protocol

For each workload and CUDA configuration:

- Build and optimize a fresh CPU optimizer and a fresh CUDA optimizer.
- Run one unmeasured CPU/GPU warm-up pair.
- Run five measured pairs, alternating CPU-first and GPU-first order.
- Include optimizer construction and optimization in external wall time.
- Exclude dataset loading and graph construction.
- Disable CUDA-to-CPU fallback.
- Retain the existing direct-solver objective gate of `1e-8`.
- Retain the matrix-campaign PCG settings: relative residual tolerance `1e-6`,
  at most 5,000 iterations, block-Jacobi preconditioning, warm start, no cap
  hits or breakdowns, and final-objective tolerance `1e-3`.

The campaign emits atomic JSON and CSV artifacts for each of
`cudss-auto`, `cudss-gtsam`, and `pcg`, including raw repetitions, medians,
system sizes, transfers, solver statistics, and detailed stage timings.

## Harness Changes

Add a stereo workload kind and three explicit workload specifications to
`timeCudaSparseLM`. Keep parsing and construction local to the timing harness;
no public GTSAM API or CUDA optimizer behavior changes are required.

The loader must reject missing or malformed calibration, pose, or factor data;
duplicate pose keys; observations referencing missing poses; and empty graphs
or values. Landmark initialization happens once per landmark. Workload names
must participate in the existing duplicate and unknown-name CLI validation.

## Validation

Use test-first development:

1. Extend the harness self-test so the new names are initially rejected by the
   old implementation, then accepted after implementation.
2. Verify the self-test and Release benchmark target build.
3. Run a one-repeat cuDSS smoke test on the smallest stereo graph with fallback
   disabled and confirm CPU/GPU objective parity.
4. Run one-repeat smoke tests for GTSAM ordering and PCG; require an applied
   direct ordering and converged PCG statistics, respectively.
5. Run the full one-warm-up/five-repeat matrix.
6. Inspect all JSON/CSV outputs for nine workload/configuration results, finite
   timings, parity-gate success, no fallback, and no PCG cap or breakdown.

## Deliverables

- The focused benchmark-harness source change and its self-test coverage.
- Machine-readable JSON and CSV artifacts for all nine combinations.
- A concise result table with CPU and GPU medians, speedups, objectives,
  iteration counts, and correctness caveats suitable for the final report.
