# CUDA GTSAM-Native BAL LM Design

## Goal

Build a first full CUDA Levenberg-Marquardt path for `timeSFMBAL` that accelerates BAL projection graphs while preserving GTSAM semantics as much as practical. The implementation should be fast enough to benchmark against PyPose on large graphs, but its abstractions should also migrate toward a general CUDA factor graph backend.

## Scope

This design covers a v0 CUDA optimizer path for BAL/SFM projection factors:

- analytic CUDA Jacobians in production,
- numeric Jacobians only as test oracles,
- `PinholeCamera<Cal3Bundler>` and `Point3` variables,
- `GeneralSFMFactor<PinholeCamera<Cal3Bundler>, Point3>` residual semantics,
- full sparse normal equations,
- cuDSS linear solves,
- unit-noise factors only,
- explicit `timeSFMBAL --cuda-lm <BALfile>` mode,
- host download of the optimized result.

This design does not include Schur complement elimination, arbitrary noise models, arbitrary factor types, incremental updates, or a public stable CUDA factor API.

## Chosen Architecture

The CUDA path should be a GTSAM-semantic implementation, not a Snavely-only BAL implementation. Device variables use packed CUDA layouts, but their residuals and update rules are chosen to match the corresponding GTSAM types.

The data flow is:

```text
SfmData
  -> pack cameras/points into DeviceValues
  -> build CudaSfmProjectionBatch observations
  -> build projection-specific CSR sparsity
  -> run CUDA LM with cuDSS
  -> download optimized cameras/points to host GTSAM-side objects
```

## Device Variable Model

Use a non-SFM-specific device camera type for `PinholeCamera<Cal3Bundler>`:

```cpp
struct DevicePinholeCameraCal3Bundler {
  double R[9];   // GTSAM pose rotation matrix
  double t[3];   // GTSAM pose translation
  double f;
  double k1;
  double k2;
};
```

Use a similarly general point type:

```cpp
struct DevicePoint3 {
  double x;
  double y;
  double z;
};
```

`DeviceValues` stores these as typed blocks. BAL/SFM code should own observation packing and projection batch construction, but the variable representation itself should not be named as SFM-only.

The existing OpenGL angle-axis `CudaCamera9` layout is not the production layout for this design. It can be removed or renamed if a Snavely-specific path is ever kept separately.

## Residual Semantics

Projection residuals should match unit-noise `GeneralSFMFactor<PinholeCamera<Cal3Bundler>, Point3>`:

```text
residual = predicted - measured
```

The device camera is stored in GTSAM convention. Packing from `SfmData` should upload the GTSAM camera pose and calibration directly. The CUDA projection code should compute the same model as `PinholeCamera<Cal3Bundler>::project2`, including `Cal3Bundler` radial distortion semantics.

## Jacobians

Production uses analytic CUDA Jacobians. Numeric Jacobians are only for tests.

There are two linearization surfaces:

1. Explicit debug/test linearization:

```text
residuals:        2 per observation
cameraJacobians:  2 x 9 per observation
pointJacobians:   2 x 3 per observation
```

2. Fused production accumulation:

```text
for each observation:
  compute residual and analytic Jacobians
  atomically accumulate J^T J into CSR values
  atomically accumulate -J^T r into RHS
```

The explicit path is for correctness checks and inspection. The fused path is for LM iterations, avoiding unnecessary global memory traffic for large graphs.

## Update Rule

The optimizer update should mirror the GTSAM types.

For `DevicePinholeCameraCal3Bundler`, the tangent vector has dimension 9:

```text
delta[0:6] -> Pose3 retract
delta[6:9] -> Cal3Bundler retract
```

This matches the intent of:

```cpp
PinholeCamera<Cal3Bundler> updated = camera.retract(delta);
```

On CUDA, implement the pose part with a device `SE(3)` exponential/retract and compose the current pose with the small pose update. The calibration part is additive:

```text
f  += delta[6]
k1 += delta[7]
k2 += delta[8]
```

For `DevicePoint3`, the update is additive:

```text
point += delta[0:3]
```

This is the first version of a future device-value contract: each type provides layout, tangent dimension, packing, download, and retract behavior.

## Sparse System

Use full sparse normal equations:

```text
A = J^T J + lambda * D
b = -J^T r
A * delta = b
```

`CudaBalCsrStructure` can stay projection-specific for v0. It should build the upper-triangle CSR pattern for camera-camera, point-point, and camera-point blocks implied by the BAL projection factors.

`DeviceSparseNormalEquations` owns device CSR arrays:

```text
row pointers
column indices
values
rhs
```

The LM iteration zeroes `values` and `rhs`, performs fused accumulation, adds damping to diagonal entries, and solves with cuDSS.

## cuDSS Solver

The first cuDSS wrapper should target symmetric positive definite normal equations with upper-triangle CSR storage. It should run analysis, factorization, and solve phases for the current matrix.

The design should leave room for future options:

- reuse symbolic analysis when the sparsity pattern is unchanged,
- expose cuDSS factorization/reordering settings,
- add symmetric-indefinite or general fallback modes,
- support full CSR if a future backend needs non-symmetric systems.

## LM Loop

Add explicit CLI mode:

```bash
timeSFMBAL --cuda-lm <BALfile>
```

The flow is:

```text
pack SfmData into DeviceValues
build projection batch
build CSR pattern
compute initial GPU error

for each iteration:
  zero normal equations
  fused-accumulate projection factors
  add LM damping
  solve with cuDSS
  retract into trial DeviceValues
  compute trial GPU error
  accept/reject step
  update lambda
  stop on max iterations or relative error tolerance

download final DeviceValues to host objects
print timing, final error, iterations, accepted steps
```

The user-facing mode is full LM only. There should be no public `--cuda-one-step` mode.

## Testing Strategy

Keep tests compact and in the existing CUDA test areas.

Core tests:

- packing test: `DevicePinholeCameraCal3Bundler` stores GTSAM-convention pose and calibration,
- residual test: CUDA residuals match `GeneralSFMFactor<PinholeCamera<Cal3Bundler>, Point3>` on tiny data,
- Jacobian test: analytic CUDA Jacobians match host numeric Jacobians in the GTSAM camera chart,
- accumulation test: device CSR normal equations match a host dense reference on tiny data,
- solver test: cuDSS solves a small SPD CSR system,
- LM smoke test: CUDA LM reduces error on a tiny BAL-like graph and downloads a valid host result,
- timing smoke test: `timeSFMBAL --cuda-lm <small BAL file>` runs and reports CUDA LM summary.

The draft test currently in `gtsam/slam/tests/testCudaSfm.cpp` is only a reference. It still assumes the old OpenGL angle-axis camera layout and must be rewritten around `DevicePinholeCameraCal3Bundler`.

## Migration Path

The v0 design is intentionally narrow in factor support but general in shape.

Future generalization should move from:

```text
CudaSfmProjectionBatch + DevicePinholeCameraCal3Bundler + DevicePoint3
```

to:

```text
DeviceFactorBatch + registered device variable layouts + registered device factor kernels
```

The key abstraction to preserve is not the BAL projection batch itself. The key abstraction is:

```text
typed packed variable blocks
per-type retract
per-factor analytic Jacobian/accumulation kernels
shared sparse normal-equation solve path
```
