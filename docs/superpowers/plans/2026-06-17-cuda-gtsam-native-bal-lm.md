# CUDA GTSAM-Native BAL LM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement an explicit `timeSFMBAL --cuda-lm` path that runs full CUDA Levenberg-Marquardt for BAL projection graphs with GTSAM-style camera storage, residuals, Jacobians, updates, sparse normal equations, and cuDSS solves.

**Architecture:** Store `PinholeCamera<Cal3Bundler>` as a packed device camera in GTSAM convention, with `Pose3` matrix pose plus `Cal3Bundler` scalars. Projection batches remain BAL/SFM-specific, but variable layouts and update rules are shaped like general device value types. Production LM uses fused analytic Jacobian accumulation into full sparse normal equations; explicit Jacobian materialization exists for tests and debugging.

**Tech Stack:** C++17, CUDA C++, GTSAM geometry/nonlinear/slam, `cuda_runtime_api`, cuDSS 0.8, CppUnitLite, CMake `GTSAM_ENABLE_CUDA` and `GTSAM_ENABLE_CUDSS`.

---

## Starting State And Guardrails

- Current branch: `cuda-sfm`.
- Approved spec: `docs/superpowers/specs/2026-06-17-cuda-gtsam-native-bal-lm-design.md`.
- Existing scratch diff: `gtsam/slam/tests/testCudaSfm.cpp` contains reference-only tests for the old OpenGL angle-axis layout. The first implementation task rewrites this file around `DevicePinholeCameraCal3Bundler`.
- Existing untracked plan: `docs/superpowers/plans/2026-06-16-cuda-bal-v0-skeleton.md`. Do not modify it unless the user asks.
- Configure command for CUDA/cuDSS work:

```bash
cmake -S . -B build-cuda-cudss-on \
  -DGTSAM_ENABLE_CUDA=ON \
  -DGTSAM_ENABLE_CUDSS=ON \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=80 \
  -DGTSAM_BUILD_TESTS=ON \
  -DGTSAM_BUILD_TIMING_ALWAYS=ON \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF
```

Expected configure result: CMake succeeds, reports `GTSAM_ENABLE_CUDA=1;GTSAM_ENABLE_CUDSS=1`, and finds cuDSS under `/usr/include/libcudss/13` and `/usr/lib/x86_64-linux-gnu/libcudss/13/libcudss.so`.

## File Structure

Create or modify these files:

- Modify `gtsam/base/cuda/CudaDeviceArray.h`: add device-to-device copy and zeroing helpers.
- Create `gtsam/nonlinear/cuda/DeviceGeometryTypes.h`: general-ish device layouts for `DevicePoint3` and `DevicePinholeCameraCal3Bundler`.
- Create `gtsam/nonlinear/cuda/DeviceGeometryKernels.h`: host/device math helpers for matrix operations, projection, analytic Jacobians, and retract.
- Modify `gtsam/nonlinear/cuda/DeviceValues.h`: store host keys per typed block and expose block copy helpers needed by LM.
- Modify `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`: expose zeroing and dimensions needed by kernels and cuDSS.
- Create `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.cu`: zeroing, diagonal damping, and small utility kernels.
- Create `gtsam/nonlinear/cuda/CudssLinearSolver.h`.
- Create `gtsam/nonlinear/cuda/CudssLinearSolver.cpp`.
- Modify `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`: tests for array helpers, sparse system helpers, and cuDSS.
- Modify `gtsam/slam/cuda/CudaSfmTypes.h`: keep observation type and projection-batch constants only; remove old camera/point storage from the SFM namespace.
- Modify `gtsam/slam/cuda/CudaSfmValues.h`: pack/unpack `SfmData` using `DevicePinholeCameraCal3Bundler` and `DevicePoint3`.
- Modify `gtsam/slam/cuda/CudaSfmProjectionBatch.h`: stop negating the measurement y coordinate; preserve `GeneralSFMFactor` measurement semantics.
- Create `gtsam/slam/cuda/CudaSfmProjectionLinearization.h`.
- Create `gtsam/slam/cuda/CudaSfmProjectionLinearization.cu`.
- Create `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h`.
- Create `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu`.
- Modify `gtsam/slam/tests/testCudaSfm.cpp`: compact tests for packing, residuals, Jacobians, accumulation, and LM.
- Modify `timing/timeSFMBAL.cpp`: add explicit `--cuda-lm`.
- Modify `timing/timeSFMBAL.h` only if a host helper needs to be shared with CUDA timing code.

The `.cu` files are picked up by the existing `gtsam/CMakeLists.txt` glob when `GTSAM_ENABLE_CUDA=ON`.

---

### Task 1: Device Geometry Types And GTSAM-Convention Packing

**Files:**
- Create: `gtsam/nonlinear/cuda/DeviceGeometryTypes.h`
- Modify: `gtsam/nonlinear/cuda/DeviceValues.h`
- Modify: `gtsam/slam/cuda/CudaSfmTypes.h`
- Modify: `gtsam/slam/cuda/CudaSfmValues.h`
- Modify: `gtsam/slam/cuda/CudaSfmProjectionBatch.h`
- Test: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Replace the scratch packing test with a failing GTSAM-convention test**

In `gtsam/slam/tests/testCudaSfm.cpp`, remove scratch references to `CudaCamera9`, `openGL2gtsam`, `CudaSfmLinearization`, and `CudaSfmLevenbergMarquardt`. Add includes for the new type header:

```cpp
#include <gtsam/nonlinear/cuda/DeviceGeometryTypes.h>
```

Add this test beside the existing projection-batch test:

```cpp
TEST(CudaSfmValues, PacksCamerasInGtsamConvention) {
  const SfmData data = makeTinySfmData();
  CudaContext context;

  DeviceValues values = PackSfmValues(data, context.stream());
  std::vector<DevicePinholeCameraCal3Bundler> cameras;
  std::vector<DevicePoint3> points;
  values.block<DevicePinholeCameraCal3Bundler>(
            kDevicePinholeCameraCal3BundlerType)
      .values.download(&cameras, context.stream());
  values.block<DevicePoint3>(kDevicePoint3Type)
      .values.download(&points, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, cameras.size());
  EXPECT_LONGS_EQUAL(2, points.size());

  DOUBLES_EQUAL(1.0, cameras[0].R[0], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[1], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[2], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[3], 1e-12);
  DOUBLES_EQUAL(1.0, cameras[0].R[4], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[5], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[6], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[7], 1e-12);
  DOUBLES_EQUAL(1.0, cameras[0].R[8], 1e-12);
  DOUBLES_EQUAL(1.0, cameras[0].t[0], 1e-12);
  DOUBLES_EQUAL(2.0, cameras[0].t[1], 1e-12);
  DOUBLES_EQUAL(3.0, cameras[0].t[2], 1e-12);
  DOUBLES_EQUAL(100.0, cameras[0].f, 1e-12);
  DOUBLES_EQUAL(0.01, cameras[0].k1, 1e-12);
  DOUBLES_EQUAL(0.001, cameras[0].k2, 1e-12);

  DOUBLES_EQUAL(1.0, points[1].x, 1e-12);
  DOUBLES_EQUAL(0.0, points[1].y, 1e-12);
  DOUBLES_EQUAL(6.0, points[1].z, 1e-12);
}
```

Update `CudaSfmProjectionBatch.PacksOnlyTracksWithAtLeastTwoMeasurements` so the first measurement expects:

```cpp
DOUBLES_EQUAL(20.0, observations[0].measuredV, 1e-12);
DOUBLES_EQUAL(21.0, observations[1].measuredV, 1e-12);
```

- [ ] **Step 2: Run the failing test**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSfm -j2
```

Expected: compile fails with errors naming `DevicePinholeCameraCal3Bundler`, `kDevicePinholeCameraCal3BundlerType`, `DevicePoint3`, or `kDevicePoint3Type`.

- [ ] **Step 3: Add device geometry types**

Create `gtsam/nonlinear/cuda/DeviceGeometryTypes.h`:

```cpp
#pragma once

#include <cstdint>

namespace gtsam::cuda {

inline constexpr uint32_t kDevicePinholeCameraCal3BundlerType = 0x50434243u;
inline constexpr uint32_t kDevicePoint3Type = 0x50544e33u;

struct DevicePinholeCameraCal3Bundler {
  double R[9];  // Row-major GTSAM pose rotation matrix.
  double t[3];  // GTSAM pose translation.
  double f;
  double k1;
  double k2;
};

struct DevicePoint3 {
  double x;
  double y;
  double z;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Preserve keys in typed value blocks**

Modify `gtsam/nonlinear/cuda/DeviceValues.h`:

```cpp
template <typename T>
struct DeviceValueBlock {
  CudaDeviceArray<T> values;
  CudaDeviceArray<double> delta;
  std::vector<Key> keys;
  int tangentDim = 0;
};
```

Inside `DeviceValues::addBlock`, after `storage->block.tangentDim = tangentDim;`, add:

```cpp
storage->block.keys = keys;
```

This host-side key vector is used by download helpers and does not move to the GPU.

- [ ] **Step 5: Move SFM types to observation-only**

Modify `gtsam/slam/cuda/CudaSfmTypes.h` so it contains only projection batch metadata:

```cpp
#pragma once

namespace gtsam::cuda {

struct CudaSfmObservation {
  int cameraSlot;
  int pointSlot;
  double measuredU;
  double measuredV;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 6: Pack SFM values into device geometry types**

Modify `gtsam/slam/cuda/CudaSfmValues.h`:

```cpp
#pragma once

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryTypes.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/sfm/SfmData.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <vector>

namespace gtsam::cuda {

inline DevicePinholeCameraCal3Bundler PackPinholeCameraCal3Bundler(
    const SfmCamera& camera) {
  const Matrix3 R = camera.pose().rotation().matrix();
  const Point3& t = camera.pose().translation();
  const Cal3Bundler& calibration = camera.calibration();

  DevicePinholeCameraCal3Bundler result{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      result.R[3 * r + c] = R(r, c);
    }
  }
  result.t[0] = t.x();
  result.t[1] = t.y();
  result.t[2] = t.z();
  result.f = calibration.fx();
  result.k1 = calibration.k1();
  result.k2 = calibration.k2();
  return result;
}

inline DevicePoint3 PackDevicePoint3(const Point3& point) {
  return {point.x(), point.y(), point.z()};
}

inline DeviceValues PackSfmValues(const SfmData& data,
                                  cudaStream_t stream = nullptr) {
  std::vector<Key> cameraKeys;
  std::vector<DevicePinholeCameraCal3Bundler> cameras;
  cameraKeys.reserve(data.numberCameras());
  cameras.reserve(data.numberCameras());
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    cameraKeys.push_back(symbol_shorthand::C(i));
    cameras.push_back(PackPinholeCameraCal3Bundler(data.camera(i)));
  }

  std::vector<Key> pointKeys;
  std::vector<DevicePoint3> points;
  pointKeys.reserve(data.numberTracks());
  points.reserve(data.numberTracks());
  for (size_t i = 0; i < data.numberTracks(); ++i) {
    pointKeys.push_back(symbol_shorthand::P(i));
    points.push_back(PackDevicePoint3(data.track(i).point3()));
  }

  DeviceValues values;
  values.addBlock<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType, 9, cameraKeys, cameras, stream);
  values.addBlock<DevicePoint3>(kDevicePoint3Type, 3, pointKeys, points,
                                stream);
  return values;
}

inline Values DownloadSfmValues(const DeviceValues& deviceValues,
                                cudaStream_t stream = nullptr) {
  std::vector<DevicePinholeCameraCal3Bundler> cameras;
  std::vector<DevicePoint3> points;
  const auto& cameraBlock =
      deviceValues.block<DevicePinholeCameraCal3Bundler>(
          kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock =
      deviceValues.block<DevicePoint3>(kDevicePoint3Type);
  cameraBlock.values.download(&cameras, stream);
  pointBlock.values.download(&points, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));

  Values result;
  for (size_t i = 0; i < cameras.size(); ++i) {
    Matrix3 R;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        R(r, c) = cameras[i].R[3 * r + c];
      }
    }
    const Pose3 pose(Rot3(R),
                     Point3(cameras[i].t[0], cameras[i].t[1],
                            cameras[i].t[2]));
    const Cal3Bundler calibration(cameras[i].f, cameras[i].k1,
                                  cameras[i].k2);
    result.insert(cameraBlock.keys[i], SfmCamera(pose, calibration));
  }
  for (size_t i = 0; i < points.size(); ++i) {
    result.insert(pointBlock.keys[i],
                  Point3(points[i].x, points[i].y, points[i].z));
  }
  return result;
}

}  // namespace gtsam::cuda
```

- [ ] **Step 7: Preserve measurement coordinates**

Modify `gtsam/slam/cuda/CudaSfmProjectionBatch.h`:

```cpp
observations.push_back(
    {static_cast<int>(measurement.first), static_cast<int>(pointSlot),
     measurement.second(0), measurement.second(1)});
```

Remove the old comment about Snavely/OpenGL y-coordinate sign conversion.

- [ ] **Step 8: Run the packing tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSfm -j2
./build-cuda-cudss-on/gtsam/slam/tests/testCudaSfm
```

Expected: existing tests and new packing tests pass.

- [ ] **Step 9: Commit**

```bash
git add gtsam/nonlinear/cuda/DeviceGeometryTypes.h \
  gtsam/nonlinear/cuda/DeviceValues.h \
  gtsam/slam/cuda/CudaSfmTypes.h \
  gtsam/slam/cuda/CudaSfmValues.h \
  gtsam/slam/cuda/CudaSfmProjectionBatch.h \
  gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: add GTSAM-convention CUDA BAL values"
```

---

### Task 2: Device Array And Sparse System Utilities

**Files:**
- Modify: `gtsam/base/cuda/CudaDeviceArray.h`
- Modify: `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`
- Create: `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.cu`
- Test: `gtsam/base/tests/testCudaBase.cpp`
- Test: `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`

- [ ] **Step 1: Add failing tests for zero and device copy**

In `gtsam/base/tests/testCudaBase.cpp`, add:

```cpp
TEST(CudaDeviceArray, ZeroesAndCopiesDeviceData) {
  CudaContext context;
  CudaDeviceArray<double> source;
  source.upload(std::vector<double>{1.0, 2.0, 3.0}, context.stream());
  source.zero(context.stream());

  CudaDeviceArray<double> target;
  target.copyFrom(source, context.stream());

  std::vector<double> actual;
  target.download(&actual, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(3, actual.size());
  DOUBLES_EQUAL(0.0, actual[0], 1e-12);
  DOUBLES_EQUAL(0.0, actual[1], 1e-12);
  DOUBLES_EQUAL(0.0, actual[2], 1e-12);
}
```

In `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`, add:

```cpp
TEST(DeviceSparseNormalEquations, ClearsValuesAndRhs) {
  CudaContext context;
  DeviceSparseNormalEquations system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3},
                       std::vector<int>{0, 1, 1}, context.stream());
  system.values().upload(std::vector<double>{1.0, 2.0, 3.0},
                         context.stream());
  system.rhs().upload(std::vector<double>{4.0, 5.0}, context.stream());

  system.zero(context.stream());

  std::vector<double> values;
  std::vector<double> rhs;
  system.values().download(&values, context.stream());
  system.rhs().download(&rhs, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(0.0, values[0], 1e-12);
  DOUBLES_EQUAL(0.0, values[1], 1e-12);
  DOUBLES_EQUAL(0.0, values[2], 1e-12);
  DOUBLES_EQUAL(0.0, rhs[0], 1e-12);
  DOUBLES_EQUAL(0.0, rhs[1], 1e-12);
}
```

- [ ] **Step 2: Run the failing tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaBase testCudaDeviceValues -j2
```

Expected: compile fails because `CudaDeviceArray::zero`, `CudaDeviceArray::copyFrom`, and `DeviceSparseNormalEquations::zero` do not exist.

- [ ] **Step 3: Implement array helpers**

Modify `gtsam/base/cuda/CudaDeviceArray.h`:

```cpp
  void zero(cudaStream_t stream = nullptr) {
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemsetAsync(data_, 0, sizeof(T) * size_, stream));
  }

  void copyFrom(const CudaDeviceArray<T>& other,
                cudaStream_t stream = nullptr) {
    resize(other.size());
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(data_, other.data(),
                                     sizeof(T) * size_,
                                     cudaMemcpyDeviceToDevice, stream));
  }
```

- [ ] **Step 4: Implement sparse system zeroing**

Modify `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`:

```cpp
  void zero(cudaStream_t stream = nullptr) {
    values_.zero(stream);
    rhs_.zero(stream);
  }
```

- [ ] **Step 5: Run tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaBase testCudaDeviceValues -j2
./build-cuda-cudss-on/gtsam/base/tests/testCudaBase
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaDeviceValues
```

Expected: both test binaries pass.

- [ ] **Step 6: Commit**

```bash
git add gtsam/base/cuda/CudaDeviceArray.h \
  gtsam/base/tests/testCudaBase.cpp \
  gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h \
  gtsam/nonlinear/tests/testCudaDeviceValues.cpp
git commit -m "feat: add CUDA device array and sparse system utilities"
```

---

### Task 3: Device Projection And Analytic Jacobian Helpers

**Files:**
- Create: `gtsam/nonlinear/cuda/DeviceGeometryKernels.h`
- Create: `gtsam/slam/cuda/CudaSfmProjectionLinearization.h`
- Create: `gtsam/slam/cuda/CudaSfmProjectionLinearization.cu`
- Test: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add failing residual and Jacobian tests**

In `gtsam/slam/tests/testCudaSfm.cpp`, add these helper functions in the anonymous namespace:

```cpp
using BundlerCamera = PinholeCamera<Cal3Bundler>;

SfmData makeTinyOptimizableBalData() {
  SfmData data;
  const std::vector<SfmCamera> trueCameras = {
      SfmCamera(Pose3(Rot3(), Point3(0.0, 0.0, 0.0)),
                Cal3Bundler(120.0, 0.0, 0.0)),
      SfmCamera(Pose3(Rot3::RzRyRx(0.0, 0.03, 0.0),
                      Point3(0.5, 0.0, 0.0)),
                Cal3Bundler(120.0, 0.0, 0.0))};

  data.cameras.emplace_back(
      Pose3(Rot3::RzRyRx(0.003, -0.002, 0.001),
            Point3(0.02, -0.01, 0.03)),
      Cal3Bundler(118.0, 1e-5, 1e-6));
  data.cameras.emplace_back(
      Pose3(Rot3::RzRyRx(0.001, 0.026, -0.002),
            Point3(0.48, 0.02, -0.01)),
      Cal3Bundler(122.0, -1e-5, 1e-6));

  const std::vector<Point3> truePoints = {
      Point3(-0.5, -0.2, 4.0), Point3(0.4, -0.1, 4.6),
      Point3(0.1, 0.5, 5.0), Point3(-0.3, 0.3, 5.5)};
  const std::vector<Point3> initialPoints = {
      Point3(-0.47, -0.22, 4.05), Point3(0.38, -0.08, 4.55),
      Point3(0.12, 0.47, 5.08), Point3(-0.35, 0.33, 5.45)};

  for (size_t pointIndex = 0; pointIndex < truePoints.size(); ++pointIndex) {
    SfmTrack track(initialPoints[pointIndex]);
    for (size_t cameraIndex = 0; cameraIndex < trueCameras.size();
         ++cameraIndex) {
      track.measurements.emplace_back(
          cameraIndex, trueCameras[cameraIndex].project(truePoints[pointIndex]));
    }
    data.tracks.push_back(track);
  }
  return data;
}

BundlerCamera toHostCamera(const DevicePinholeCameraCal3Bundler& camera) {
  Matrix3 R;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = camera.R[3 * r + c];
    }
  }
  return BundlerCamera(
      Pose3(Rot3(R), Point3(camera.t[0], camera.t[1], camera.t[2])),
      Cal3Bundler(camera.f, camera.k1, camera.k2));
}

Vector2 hostResidual(const DevicePinholeCameraCal3Bundler& camera,
                     const DevicePoint3& point,
                     const CudaSfmObservation& obs) {
  const auto model = noiseModel::Unit::Create(2);
  const GeneralSFMFactor<BundlerCamera, Point3> factor(
      Point2(obs.measuredU, obs.measuredV), model, C(0), P(0));
  return factor.evaluateError(toHostCamera(camera),
                              Point3(point.x, point.y, point.z));
}

void hostNumericJacobians(const DevicePinholeCameraCal3Bundler& camera,
                          const DevicePoint3& point,
                          const CudaSfmObservation& obs, double* cameraJ,
                          double* pointJ) {
  constexpr double kEps = 1e-6;
  const BundlerCamera hostCamera = toHostCamera(camera);
  const Point3 hostPoint(point.x, point.y, point.z);

  for (int j = 0; j < 9; ++j) {
    Vector delta = Vector::Zero(9);
    delta(j) = kEps;
    const BundlerCamera plus = hostCamera.retract(delta);
    delta(j) = -kEps;
    const BundlerCamera minus = hostCamera.retract(delta);
    const DevicePinholeCameraCal3Bundler plusDevice =
        PackPinholeCameraCal3Bundler(plus);
    const DevicePinholeCameraCal3Bundler minusDevice =
        PackPinholeCameraCal3Bundler(minus);
    const Vector2 rPlus = hostResidual(plusDevice, point, obs);
    const Vector2 rMinus = hostResidual(minusDevice, point, obs);
    cameraJ[j] = (rPlus(0) - rMinus(0)) / (2.0 * kEps);
    cameraJ[9 + j] = (rPlus(1) - rMinus(1)) / (2.0 * kEps);
  }

  for (int j = 0; j < 3; ++j) {
    DevicePoint3 plus = point;
    DevicePoint3 minus = point;
    (&plus.x)[j] += kEps;
    (&minus.x)[j] -= kEps;
    const Vector2 rPlus = hostResidual(camera, plus, obs);
    const Vector2 rMinus = hostResidual(camera, minus, obs);
    pointJ[j] = (rPlus(0) - rMinus(0)) / (2.0 * kEps);
    pointJ[3 + j] = (rPlus(1) - rMinus(1)) / (2.0 * kEps);
  }
}
```

Add this test:

```cpp
TEST(CudaSfmProjectionLinearization, MatchesGeneralSfmFactorOnTinyData) {
  const SfmData data = makeTinyOptimizableBalData();
  CudaContext context;
  DeviceValues values = PackSfmValues(data, context.stream());
  const CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  CudaSfmProjectionLinearization linearization;

  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<DevicePinholeCameraCal3Bundler> cameras;
  std::vector<DevicePoint3> points;
  std::vector<CudaSfmObservation> observations;
  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  values.block<DevicePinholeCameraCal3Bundler>(
            kDevicePinholeCameraCal3BundlerType)
      .values.download(&cameras, context.stream());
  values.block<DevicePoint3>(kDevicePoint3Type)
      .values.download(&points, context.stream());
  batch.observations().download(&observations, context.stream());
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2 * observations.size(), residuals.size());
  EXPECT_LONGS_EQUAL(18 * observations.size(), cameraJacobians.size());
  EXPECT_LONGS_EQUAL(6 * observations.size(), pointJacobians.size());

  for (size_t obsIndex = 0; obsIndex < observations.size(); ++obsIndex) {
    const auto& obs = observations[obsIndex];
    const auto& camera = cameras[obs.cameraSlot];
    const auto& point = points[obs.pointSlot];
    const Vector2 expectedResidual = hostResidual(camera, point, obs);
    DOUBLES_EQUAL(expectedResidual(0), residuals[2 * obsIndex], 1e-7);
    DOUBLES_EQUAL(expectedResidual(1), residuals[2 * obsIndex + 1], 1e-7);

    double expectedCameraJ[18];
    double expectedPointJ[6];
    hostNumericJacobians(camera, point, obs, expectedCameraJ, expectedPointJ);
    for (int i = 0; i < 18; ++i) {
      DOUBLES_EQUAL(expectedCameraJ[i], cameraJacobians[18 * obsIndex + i],
                    5e-4);
    }
    for (int i = 0; i < 6; ++i) {
      DOUBLES_EQUAL(expectedPointJ[i], pointJacobians[6 * obsIndex + i],
                    5e-4);
    }
  }
}
```

- [ ] **Step 2: Run the failing test**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSfm -j2
```

Expected: compile fails because `CudaSfmProjectionLinearization` and `LinearizeCudaSfmProjectionBatch` do not exist.

- [ ] **Step 3: Add projection linearization API**

Create `gtsam/slam/cuda/CudaSfmProjectionLinearization.h`:

```cpp
#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

struct CudaSfmProjectionLinearization {
  CudaDeviceArray<double> residuals;
  CudaDeviceArray<double> cameraJacobians;
  CudaDeviceArray<double> pointJacobians;
};

void LinearizeCudaSfmProjectionBatch(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    CudaSfmProjectionLinearization* linearization,
    cudaStream_t stream = nullptr);

double ComputeCudaSfmProjectionError(const DeviceValues& values,
                                     const CudaSfmProjectionBatch& batch,
                                     cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Add device geometry helpers**

Create `gtsam/nonlinear/cuda/DeviceGeometryKernels.h` with row-major matrix helpers and analytic projection. The core formulas must match these GTSAM source formulas:

- `PinholeBase::project2`: `q = R^T * (point - t)`, `pn = (q.x/q.z, q.y/q.z)`.
- `PinholeBase::Dpose`: rows `[u*v, -1-u*u, v, -d, 0, d*u]` and `[1+v*v, -u*v, -u, 0, -d, d*v]`.
- `PinholeBase::Dpoint`: `d * [Rt.row(0) - u * Rt.row(2); Rt.row(1) - v * Rt.row(2)]`.
- `Cal3Bundler::uncalibrate`: `r = x*x + y*y`, `g = 1 + (k1 + k2*r)*r`, pixel `(f*g*x, f*g*y)`.

The helper signature should be:

```cpp
namespace gtsam::cuda {

struct DeviceProjectionResult {
  double residual[2];
  double cameraJacobian[18];  // row-major 2 x 9
  double pointJacobian[6];    // row-major 2 x 3
};

__host__ __device__ DeviceProjectionResult EvaluatePinholeBundlerProjection(
    const DevicePinholeCameraCal3Bundler& camera,
    const DevicePoint3& point,
    const CudaSfmObservation& observation);

__host__ __device__ DevicePinholeCameraCal3Bundler RetractCamera(
    const DevicePinholeCameraCal3Bundler& camera, const double* delta9);

__host__ __device__ DevicePoint3 RetractPoint(const DevicePoint3& point,
                                               const double* delta3);

}  // namespace gtsam::cuda
```

Use `__host__ __device__` so tests can reuse the math on host if needed.

- [ ] **Step 5: Implement explicit CUDA linearization**

Create `gtsam/slam/cuda/CudaSfmProjectionLinearization.cu`:

```cpp
#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>

namespace gtsam::cuda {
namespace {

__global__ void LinearizeKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const CudaSfmObservation* observations, int numObservations,
    double* residuals, double* cameraJacobians, double* pointJacobians) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= numObservations) return;

  const CudaSfmObservation obs = observations[i];
  const DeviceProjectionResult result =
      EvaluatePinholeBundlerProjection(cameras[obs.cameraSlot],
                                       points[obs.pointSlot], obs);

  residuals[2 * i] = result.residual[0];
  residuals[2 * i + 1] = result.residual[1];
  for (int k = 0; k < 18; ++k) {
    cameraJacobians[18 * i + k] = result.cameraJacobian[k];
  }
  for (int k = 0; k < 6; ++k) {
    pointJacobians[6 * i + k] = result.pointJacobian[k];
  }
}

}  // namespace

void LinearizeCudaSfmProjectionBatch(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    CudaSfmProjectionLinearization* linearization, cudaStream_t stream) {
  const int n = static_cast<int>(batch.numObservations());
  linearization->residuals.resize(2 * static_cast<size_t>(n));
  linearization->cameraJacobians.resize(18 * static_cast<size_t>(n));
  linearization->pointJacobians.resize(6 * static_cast<size_t>(n));
  if (n == 0) return;

  const auto& cameras =
      values.block<DevicePinholeCameraCal3Bundler>(
          kDevicePinholeCameraCal3BundlerType)
          .values;
  const auto& points = values.block<DevicePoint3>(kDevicePoint3Type).values;

  constexpr int kBlockSize = 256;
  const int grid = (n + kBlockSize - 1) / kBlockSize;
  LinearizeKernel<<<grid, kBlockSize, 0, stream>>>(
      cameras.data(), points.data(), batch.observations().data(), n,
      linearization->residuals.data(), linearization->cameraJacobians.data(),
      linearization->pointJacobians.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

double ComputeCudaSfmProjectionError(const DeviceValues& values,
                                     const CudaSfmProjectionBatch& batch,
                                     cudaStream_t stream) {
  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization, stream);
  std::vector<double> residuals;
  linearization.residuals.download(&residuals, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  double error = 0.0;
  for (double r : residuals) error += 0.5 * r * r;
  return error;
}

}  // namespace gtsam::cuda
```

- [ ] **Step 6: Run tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSfm -j2
./build-cuda-cudss-on/gtsam/slam/tests/testCudaSfm
```

Expected: residual/Jacobian test passes within the declared tolerances.

- [ ] **Step 7: Commit**

```bash
git add gtsam/nonlinear/cuda/DeviceGeometryKernels.h \
  gtsam/slam/cuda/CudaSfmProjectionLinearization.h \
  gtsam/slam/cuda/CudaSfmProjectionLinearization.cu \
  gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: linearize CUDA projection factors analytically"
```

---

### Task 4: Fused Sparse Normal-Equation Accumulation

**Files:**
- Modify: `gtsam/slam/cuda/CudaSfmProjectionLinearization.h`
- Modify: `gtsam/slam/cuda/CudaSfmProjectionLinearization.cu`
- Modify: `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`
- Test: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add failing accumulation test**

Add this declaration to `gtsam/slam/cuda/CudaSfmProjectionLinearization.h`:

```cpp
void AccumulateCudaSfmNormalEquations(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, DeviceSparseNormalEquations* system,
    cudaStream_t stream = nullptr);
```

Add to `gtsam/slam/tests/testCudaSfm.cpp`:

```cpp
TEST(CudaSfmNormalEquations, AccumulatesTinyDenseSystem) {
  const SfmData data = makeTinyOptimizableBalData();
  CudaContext context;
  DeviceValues values = PackSfmValues(data, context.stream());
  const CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());

  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);
  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());
  AccumulateCudaSfmNormalEquations(values, batch,
                                   static_cast<int>(structure.numCameras()),
                                   &system, context.stream());

  std::vector<CudaSfmObservation> observations;
  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  std::vector<double> valuesHost;
  std::vector<double> rhsHost;
  batch.observations().download(&observations, context.stream());
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  system.values().download(&valuesHost, context.stream());
  system.rhs().download(&rhsHost, context.stream());
  context.synchronize();

  std::vector<double> expectedDense(
      static_cast<size_t>(structure.dimension()) * structure.dimension(), 0.0);
  std::vector<double> expectedRhs(static_cast<size_t>(structure.dimension()),
                                  0.0);

  for (size_t obsIndex = 0; obsIndex < observations.size(); ++obsIndex) {
    const CudaSfmObservation& obs = observations[obsIndex];
    const int cameraBase = 9 * obs.cameraSlot;
    const int pointBase = static_cast<int>(9 * structure.numCameras() +
                                           3 * obs.pointSlot);
    double j[2][12] = {};
    for (int k = 0; k < 9; ++k) {
      j[0][k] = cameraJacobians[18 * obsIndex + k];
      j[1][k] = cameraJacobians[18 * obsIndex + 9 + k];
    }
    for (int k = 0; k < 3; ++k) {
      j[0][9 + k] = pointJacobians[6 * obsIndex + k];
      j[1][9 + k] = pointJacobians[6 * obsIndex + 3 + k];
    }
    int global[12];
    for (int k = 0; k < 9; ++k) global[k] = cameraBase + k;
    for (int k = 0; k < 3; ++k) global[9 + k] = pointBase + k;

    for (int a = 0; a < 12; ++a) {
      expectedRhs[global[a]] += -j[0][a] * residuals[2 * obsIndex];
      expectedRhs[global[a]] += -j[1][a] * residuals[2 * obsIndex + 1];
      for (int b = a; b < 12; ++b) {
        const double contribution = j[0][a] * j[0][b] + j[1][a] * j[1][b];
        const int row = std::min(global[a], global[b]);
        const int col = std::max(global[a], global[b]);
        expectedDense[static_cast<size_t>(row) * structure.dimension() + col] +=
            contribution;
      }
    }
  }

  for (int row = 0; row < structure.dimension(); ++row) {
    DOUBLES_EQUAL(expectedRhs[row], rhsHost[row], 1e-6);
    for (int k = structure.rowPointers()[row];
         k < structure.rowPointers()[row + 1]; ++k) {
      const int col = structure.colIndices()[k];
      DOUBLES_EQUAL(
          expectedDense[static_cast<size_t>(row) * structure.dimension() + col],
          valuesHost[k], 1e-6);
    }
  }
}
```

- [ ] **Step 2: Run the failing test**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSfm -j2
```

Expected: compile fails because `AccumulateCudaSfmNormalEquations` is declared but not defined.

- [ ] **Step 3: Implement CSR lookup and fused accumulation**

In `CudaSfmProjectionLinearization.cu`, add:

```cpp
__device__ int FindCsrEntry(const int* rowPointers, const int* colIndices,
                            int row, int col) {
  int begin = rowPointers[row];
  int end = rowPointers[row + 1];
  while (begin < end) {
    const int mid = begin + (end - begin) / 2;
    const int value = colIndices[mid];
    if (value < col) {
      begin = mid + 1;
    } else {
      end = mid;
    }
  }
  return begin;
}
```

Add a kernel that evaluates each observation once and atomically adds:

```cpp
// RHS
atomicAdd(&rhs[global[a]], -jac[0][a] * residual0 - jac[1][a] * residual1);

// Upper-triangle Hessian
const int row = min(global[a], global[b]);
const int col = max(global[a], global[b]);
const int entry = FindCsrEntry(rowPointers, colIndices, row, col);
atomicAdd(&values[entry], jac[0][a] * jac[0][b] + jac[1][a] * jac[1][b]);
```

The 12 local variables are ordered as camera 0..8, point 0..2. Global indices are:

```cpp
cameraBase = 9 * obs.cameraSlot;
pointBase = 9 * numCameras + 3 * obs.pointSlot;
```

- [ ] **Step 4: Zero the system before accumulation**

At the top of `AccumulateCudaSfmNormalEquations`, call:

```cpp
system->zero(stream);
```

Then launch `AccumulateKernel` with 256 threads per block.

- [ ] **Step 5: Run tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSfm -j2
./build-cuda-cudss-on/gtsam/slam/tests/testCudaSfm
```

Expected: accumulation test passes.

- [ ] **Step 6: Commit**

```bash
git add gtsam/slam/cuda/CudaSfmProjectionLinearization.h \
  gtsam/slam/cuda/CudaSfmProjectionLinearization.cu \
  gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h \
  gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: accumulate CUDA projection normal equations"
```

---

### Task 5: Diagonal Damping And cuDSS Solver

**Files:**
- Modify: `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`
- Create: `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.cu`
- Create: `gtsam/nonlinear/cuda/CudssLinearSolver.h`
- Create: `gtsam/nonlinear/cuda/CudssLinearSolver.cpp`
- Test: `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`

- [ ] **Step 1: Add failing sparse solve test**

In `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`, add:

```cpp
#if GTSAM_ENABLE_CUDSS
#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>
#endif
```

Add:

```cpp
TEST(DeviceSparseNormalEquations, AddsDiagonalDamping) {
  CudaContext context;
  DeviceSparseNormalEquations system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3},
                       std::vector<int>{0, 1, 1}, context.stream());
  system.values().upload(std::vector<double>{2.0, 0.5, 3.0},
                         context.stream());
  system.addDiagonalDamping(0.25, context.stream());

  std::vector<double> values;
  system.values().download(&values, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(2.25, values[0], 1e-12);
  DOUBLES_EQUAL(0.5, values[1], 1e-12);
  DOUBLES_EQUAL(3.25, values[2], 1e-12);
}

#if GTSAM_ENABLE_CUDSS
TEST(CudssLinearSolver, SolvesSmallSpdSystem) {
  CudaContext context;
  DeviceSparseNormalEquations system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3},
                       std::vector<int>{0, 1, 1}, context.stream());
  system.values().upload(std::vector<double>{4.0, 1.0, 3.0},
                         context.stream());
  system.rhs().upload(std::vector<double>{1.0, 2.0}, context.stream());

  CudaDeviceArray<double> solution;
  CudssLinearSolver solver;
  solver.solveSpd(system, &solution, context.stream());

  std::vector<double> actual;
  solution.download(&actual, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(1.0 / 11.0, actual[0], 1e-10);
  DOUBLES_EQUAL(7.0 / 11.0, actual[1], 1e-10);
}
#endif
```

- [ ] **Step 2: Run failing tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaDeviceValues -j2
```

Expected: compile fails because `addDiagonalDamping` and `CudssLinearSolver` do not exist.

- [ ] **Step 3: Add damping API**

Modify `DeviceSparseNormalEquations.h`:

```cpp
  void addDiagonalDamping(double lambda, cudaStream_t stream = nullptr);
```

Create `DeviceSparseNormalEquations.cu`:

```cpp
#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>

namespace gtsam::cuda {
namespace {

__global__ void AddDiagonalDampingKernel(int rows, double lambda,
                                         const int* rowPointers,
                                         const int* colIndices,
                                         double* values) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows) return;
  for (int k = rowPointers[row]; k < rowPointers[row + 1]; ++k) {
    if (colIndices[k] == row) {
      values[k] += lambda;
      return;
    }
  }
}

}  // namespace

void DeviceSparseNormalEquations::addDiagonalDamping(double lambda,
                                                     cudaStream_t stream) {
  if (rows_ == 0) return;
  constexpr int kBlockSize = 256;
  const int grid = (rows_ + kBlockSize - 1) / kBlockSize;
  AddDiagonalDampingKernel<<<grid, kBlockSize, 0, stream>>>(
      rows_, lambda, rowPointers_.data(), colIndices_.data(), values_.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Add cuDSS wrapper**

Create `gtsam/nonlinear/cuda/CudssLinearSolver.h`:

```cpp
#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

class CudssLinearSolver {
 public:
  void solveSpd(const DeviceSparseNormalEquations& system,
                CudaDeviceArray<double>* solution,
                cudaStream_t stream = nullptr) const;
};

}  // namespace gtsam::cuda
```

Create `gtsam/nonlinear/cuda/CudssLinearSolver.cpp`:

```cpp
#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>

#if GTSAM_ENABLE_CUDSS
#include <cudss.h>
#endif

#include <sstream>
#include <stdexcept>

namespace gtsam::cuda {

#if GTSAM_ENABLE_CUDSS
namespace {

void CheckCudss(cudssStatus_t status, const char* expression) {
  if (status == CUDSS_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuDSS call failed: " << expression << " status=" << status;
  throw std::runtime_error(os.str());
}

struct CudssMatrixHandle {
  cudssMatrix_t matrix = nullptr;
  ~CudssMatrixHandle() {
    if (matrix) cudssMatrixDestroy(matrix);
  }
};

}  // namespace
#endif

void CudssLinearSolver::solveSpd(const DeviceSparseNormalEquations& system,
                                 CudaDeviceArray<double>* solution,
                                 cudaStream_t stream) const {
#if !GTSAM_ENABLE_CUDSS
  throw std::runtime_error("CudssLinearSolver requires GTSAM_ENABLE_CUDSS=ON");
#else
  const int rows = system.rows();
  solution->resize(static_cast<size_t>(rows));

  cudssHandle_t handle = nullptr;
  cudssConfig_t config = nullptr;
  cudssData_t data = nullptr;
  CheckCudss(cudssCreate(&handle), "cudssCreate");
  CheckCudss(cudssSetStream(handle, stream), "cudssSetStream");
  CheckCudss(cudssConfigCreate(&config), "cudssConfigCreate");
  CheckCudss(cudssDataCreate(handle, &data), "cudssDataCreate");

  CudssMatrixHandle A;
  CudssMatrixHandle X;
  CudssMatrixHandle B;
  CheckCudss(cudssMatrixCreateCsr(
                 &A.matrix, rows, rows, system.nonzeros(),
                 system.rowPointers().data(), system.rowPointers().data() + 1,
                 system.colIndices().data(), system.values().data(),
                 CUDSS_R_32I, CUDSS_R_32I, CUDSS_R_64F, CUDSS_MTYPE_SPD,
                 CUDSS_MVIEW_UPPER, CUDSS_BASE_ZERO),
             "cudssMatrixCreateCsr");
  CheckCudss(cudssMatrixCreateDn(&X.matrix, rows, 1, rows, solution->data(),
                                 CUDSS_R_64F, CUDSS_LAYOUT_COL_MAJOR),
             "cudssMatrixCreateDn(X)");
  CheckCudss(cudssMatrixCreateDn(&B.matrix, rows, 1, rows, system.rhs().data(),
                                 CUDSS_R_64F, CUDSS_LAYOUT_COL_MAJOR),
             "cudssMatrixCreateDn(B)");

  CheckCudss(cudssExecute(handle, CUDSS_PHASE_ANALYSIS, config, data, A.matrix,
                          X.matrix, B.matrix),
             "cudssExecute(analysis)");
  CheckCudss(cudssExecute(handle, CUDSS_PHASE_FACTORIZATION, config, data,
                          A.matrix, X.matrix, B.matrix),
             "cudssExecute(factorization)");
  CheckCudss(cudssExecute(handle, CUDSS_PHASE_SOLVE, config, data, A.matrix,
                          X.matrix, B.matrix),
             "cudssExecute(solve)");

  CheckCudss(cudssDataDestroy(handle, data), "cudssDataDestroy");
  CheckCudss(cudssConfigDestroy(config), "cudssConfigDestroy");
  CheckCudss(cudssDestroy(handle), "cudssDestroy");
#endif
}

}  // namespace gtsam::cuda
```

- [ ] **Step 5: Run tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaDeviceValues -j2
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaDeviceValues
```

Expected: damping and cuDSS small solve tests pass.

- [ ] **Step 6: Commit**

```bash
git add gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h \
  gtsam/nonlinear/cuda/DeviceSparseNormalEquations.cu \
  gtsam/nonlinear/cuda/CudssLinearSolver.h \
  gtsam/nonlinear/cuda/CudssLinearSolver.cpp \
  gtsam/nonlinear/tests/testCudaDeviceValues.cpp
git commit -m "feat: add cuDSS SPD solver wrapper"
```

---

### Task 6: CUDA LM Loop, Retract, Cost, And Download

**Files:**
- Modify: `gtsam/nonlinear/cuda/DeviceGeometryKernels.h`
- Create: `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h`
- Create: `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu`
- Test: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add failing LM smoke test**

Add include:

```cpp
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
```

Add test:

```cpp
#if GTSAM_ENABLE_CUDSS
TEST(CudaSfmLevenbergMarquardt, ReducesTinyBalErrorAndDownloadsValues) {
  const SfmData data = makeTinyOptimizableBalData();
  CudaSfmLevenbergMarquardtParams params;
  params.maxIterations = 5;
  params.relativeErrorTol = 1e-12;
  params.initialLambda = 1e-3;

  const CudaSfmLevenbergMarquardtResult result =
      OptimizeCudaSfm(data, params);

  CHECK(result.iterations > 0);
  CHECK(result.acceptedSteps > 0);
  CHECK(result.finalError < result.initialError);
  CHECK(result.optimizedValues.exists(C(0)));
  CHECK(result.optimizedValues.exists(P(0)));
  const auto& camera0 = result.optimizedValues.at<SfmCamera>(C(0));
  const auto& point0 = result.optimizedValues.at<Point3>(P(0));
  CHECK(camera0.calibration().fx() > 0.0);
  CHECK(std::isfinite(point0.x()));
}
#endif
```

- [ ] **Step 2: Run failing test**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSfm -j2
```

Expected: compile fails because `CudaSfmLevenbergMarquardt.h` and `OptimizeCudaSfm` do not exist.

- [ ] **Step 3: Add LM API**

Create `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h`:

```cpp
#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>

namespace gtsam::cuda {

struct CudaSfmLevenbergMarquardtParams {
  int maxIterations = 20;
  double initialLambda = 1e-3;
  double lambdaUpFactor = 10.0;
  double lambdaDownFactor = 0.1;
  double relativeErrorTol = 1e-5;
};

struct CudaSfmLevenbergMarquardtResult {
  double initialError = 0.0;
  double finalError = 0.0;
  int iterations = 0;
  int acceptedSteps = 0;
  Values optimizedValues;
};

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const CudaSfmLevenbergMarquardtParams& params);

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Implement device retract kernels**

In `DeviceGeometryKernels.h`, implement `RetractCamera` with this exact update convention:

```text
updated_pose = current_pose.compose(Pose3::Expmap(delta_pose))
updated_calibration = calibration + delta_calibration
```

Use the SE(3) expmap formulas from `gtsam/geometry/Pose3.cpp`:

```text
w = delta[0:3]
v = delta[3:6]
R_delta = SO3Expmap(w)
t_delta = J_left_SO3(w) * v
R_new = R_current * R_delta
t_new = t_current + R_current * t_delta
```

For small `theta`, use series expansions:

```text
sin(theta)/theta ~= 1 - theta^2/6
(1-cos(theta))/theta^2 ~= 1/2 - theta^2/24
(theta-sin(theta))/theta^3 ~= 1/6 - theta^2/120
```

For `DevicePoint3`, add the three delta entries directly.

- [ ] **Step 5: Implement LM loop**

Create `CudaSfmLevenbergMarquardt.cu`:

```cpp
#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>

namespace gtsam::cuda {
namespace {

__global__ void ApplyDeltaKernel(
    const DevicePinholeCameraCal3Bundler* cameras,
    const DevicePoint3* points, int numCameras, int numPoints,
    const double* delta, DevicePinholeCameraCal3Bundler* trialCameras,
    DevicePoint3* trialPoints) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < numCameras) {
    trialCameras[i] = RetractCamera(cameras[i], delta + 9 * i);
  }
  if (i < numPoints) {
    const int pointOffset = 9 * numCameras + 3 * i;
    trialPoints[i] = RetractPoint(points[i], delta + pointOffset);
  }
}

}  // namespace

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const CudaSfmLevenbergMarquardtParams& params) {
  CudaContext context;
  DeviceValues current = PackSfmValues(data, context.stream());
  DeviceValues trial = PackSfmValues(data, context.stream());
  const CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);

  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());
  CudaDeviceArray<double> delta;
  CudssLinearSolver solver;

  CudaSfmLevenbergMarquardtResult result;
  double currentError =
      ComputeCudaSfmProjectionError(current, batch, context.stream());
  result.initialError = currentError;
  double lambda = params.initialLambda;

  for (int iteration = 0; iteration < params.maxIterations; ++iteration) {
    AccumulateCudaSfmNormalEquations(
        current, batch, static_cast<int>(structure.numCameras()), &system,
        context.stream());
    system.addDiagonalDamping(lambda, context.stream());
    solver.solveSpd(system, &delta, context.stream());

    auto& currentCameras =
        current.block<DevicePinholeCameraCal3Bundler>(
            kDevicePinholeCameraCal3BundlerType)
            .values;
    auto& currentPoints =
        current.block<DevicePoint3>(kDevicePoint3Type).values;
    auto& trialCameras =
        trial.block<DevicePinholeCameraCal3Bundler>(
            kDevicePinholeCameraCal3BundlerType)
            .values;
    auto& trialPoints = trial.block<DevicePoint3>(kDevicePoint3Type).values;

    const int numCameras = static_cast<int>(structure.numCameras());
    const int numPoints = static_cast<int>(structure.numPoints());
    constexpr int kBlockSize = 256;
    const int grid =
        (std::max(numCameras, numPoints) + kBlockSize - 1) / kBlockSize;
    ApplyDeltaKernel<<<grid, kBlockSize, 0, context.stream()>>>(
        currentCameras.data(), currentPoints.data(), numCameras, numPoints,
        delta.data(), trialCameras.data(), trialPoints.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());

    const double trialError =
        ComputeCudaSfmProjectionError(trial, batch, context.stream());
    ++result.iterations;

    if (trialError < currentError) {
      const double relativeDecrease =
          (currentError - trialError) / std::max(1.0, currentError);
      currentCameras.copyFrom(trialCameras, context.stream());
      currentPoints.copyFrom(trialPoints, context.stream());
      currentError = trialError;
      ++result.acceptedSteps;
      lambda *= params.lambdaDownFactor;
      if (relativeDecrease < params.relativeErrorTol) break;
    } else {
      lambda *= params.lambdaUpFactor;
    }
  }

  result.finalError = currentError;
  result.optimizedValues = DownloadSfmValues(current, context.stream());
  return result;
}

}  // namespace gtsam::cuda
```

Return `CudaSfmLevenbergMarquardtResult` by value. `Values` already has a move constructor, so no custom move assignment is needed for the result struct.

- [ ] **Step 6: Run LM test**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSfm -j2
./build-cuda-cudss-on/gtsam/slam/tests/testCudaSfm
```

Expected: tiny LM smoke test passes, with `finalError < initialError`.

- [ ] **Step 7: Commit**

```bash
git add gtsam/nonlinear/cuda/DeviceGeometryKernels.h \
  gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h \
  gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu \
  gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: add CUDA SFM Levenberg-Marquardt loop"
```

---

### Task 7: Wire `timeSFMBAL --cuda-lm`

**Files:**
- Modify: `timing/timeSFMBAL.cpp`
- Test: `timing/timeSFMBAL.cpp` via binary run

- [ ] **Step 1: Add failing CLI build test**

Modify `timing/timeSFMBAL.cpp` includes:

```cpp
#if GTSAM_ENABLE_CUDA
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#endif
```

Add `cudaLm` to `RunOptions`:

```cpp
bool cudaLm = false;
```

Add parsing for:

```cpp
if (strcmp(argv[i], "--cuda-lm") == 0) {
  cudaLm = true;
  continue;
}
```

Update `usage()` text:

```cpp
return "Usage: timeSFMBAL [--colamd] [--profile] [--cuda-structure-only] "
       "[--cuda-lm] [--benchmark-action-json FILE] [BALfile]";
```

- [ ] **Step 2: Run failing build**

Run:

```bash
cmake --build build-cuda-cudss-on --target timeSFMBAL -j2
```

Expected: build fails if `RunOptions` construction sites are not updated yet.

- [ ] **Step 3: Complete parser and conflicts**

Update all `RunOptions` return statements to include `cudaLm`.

Reject unsupported combinations:

```cpp
if (cudaLm && benchmarkActionJson) {
  throw runtime_error(usage());
}
if (cudaLm && cudaStructureOnly) {
  throw runtime_error(usage());
}
```

- [ ] **Step 4: Add CUDA LM execution path**

In the per-file loop after loading `SfmData db`, add:

```cpp
#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
    if (options.cudaLm) {
      gtsam::cuda::CudaSfmLevenbergMarquardtParams params;
      params.maxIterations = 20;
      params.relativeErrorTol = 0.01;

      const auto start = std::chrono::high_resolution_clock::now();
      const gtsam::cuda::CudaSfmLevenbergMarquardtResult result =
          gtsam::cuda::OptimizeCudaSfm(db, params);
      const auto end = std::chrono::high_resolution_clock::now();
      const std::chrono::duration<double> elapsed = end - start;

      std::cout << "  CUDA LM: " << elapsed.count() << " s\n";
      std::cout << "Initial error: " << std::setprecision(15)
                << result.initialError << "\n";
      std::cout << "Final error: " << result.finalError
                << ", iterations: " << result.iterations
                << ", accepted: " << result.acceptedSteps
                << std::setprecision(6) << "\n";
      continue;
    }
#elif GTSAM_ENABLE_CUDA
    if (options.cudaLm) {
      throw std::runtime_error(
          "--cuda-lm requires configuring with GTSAM_ENABLE_CUDSS=ON");
    }
#else
    if (options.cudaLm) {
      throw std::runtime_error(
          "--cuda-lm requires configuring with GTSAM_ENABLE_CUDA=ON and "
          "GTSAM_ENABLE_CUDSS=ON");
    }
#endif
```

Update the final table and benchmark JSON guards so they also skip when `options.cudaLm` is true:

```cpp
if (!options.profile && !options.cudaStructureOnly && !options.cudaLm) {
```

and:

```cpp
if (options.benchmarkActionJson && !options.cudaStructureOnly && !options.cudaLm) {
```

- [ ] **Step 5: Build and smoke run**

Run:

```bash
cmake --build build-cuda-cudss-on --target timeSFMBAL -j2
./build-cuda-cudss-on/timing/timeSFMBAL --cuda-lm examples/Data/dubrovnik-16-22106-pre.txt
```

Expected output includes:

```text
CUDA LM:
Initial error:
Final error:
iterations:
accepted:
```

- [ ] **Step 6: Commit**

```bash
git add timing/timeSFMBAL.cpp
git commit -m "feat: add timeSFMBAL CUDA LM mode"
```

---

### Task 8: Verification, Cleanup, And Benchmark Notes

**Files:**
- Modify: only files required by fixes discovered during verification

- [ ] **Step 1: Run focused CUDA tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaBase testCudaDeviceValues testCudaSfm timeSFMBAL -j2
./build-cuda-cudss-on/gtsam/base/tests/testCudaBase
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaDeviceValues
./build-cuda-cudss-on/gtsam/slam/tests/testCudaSfm
```

Expected: all three test binaries pass.

- [ ] **Step 2: Run CTest subsets**

Run:

```bash
cmake --build build-cuda-cudss-on --target check.base check.nonlinear check.slam -j2
```

Expected: `check.base`, `check.nonlinear`, and `check.slam` pass.

- [ ] **Step 3: Run timing smoke tests**

Run:

```bash
./build-cuda-cudss-on/timing/timeSFMBAL --cuda-structure-only examples/Data/dubrovnik-16-22106-pre.txt
./build-cuda-cudss-on/timing/timeSFMBAL --cuda-lm examples/Data/dubrovnik-16-22106-pre.txt
```

Expected: structure-only still prints cameras, points, observations, dimension, and CSR nonzeros. CUDA LM prints initial error, final error, iterations, and accepted steps.

- [ ] **Step 4: Check formatting and dirty state**

Run:

```bash
git diff --check
git status --short
```

Expected: `git diff --check` prints no whitespace errors. `git status --short` shows only intentionally untracked data files or documentation that the user already had untracked before implementation.

- [ ] **Step 5: Remove build artifacts if the user asks for a clean tree**

Run only if requested:

```bash
rm -rf build-cuda-cudss-on
```

Expected: build directory is removed. This repo ignores `/build-*/`, so this does not affect committed files.

- [ ] **Step 6: Final commit for verification fixes**

If verification required code changes after Task 7, commit them:

```bash
git add gtsam/base/cuda/CudaDeviceArray.h \
  gtsam/nonlinear/cuda/DeviceGeometryTypes.h \
  gtsam/nonlinear/cuda/DeviceGeometryKernels.h \
  gtsam/nonlinear/cuda/DeviceValues.h \
  gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h \
  gtsam/nonlinear/cuda/DeviceSparseNormalEquations.cu \
  gtsam/nonlinear/cuda/CudssLinearSolver.h \
  gtsam/nonlinear/cuda/CudssLinearSolver.cpp \
  gtsam/nonlinear/tests/testCudaDeviceValues.cpp \
  gtsam/slam/cuda/CudaSfmTypes.h \
  gtsam/slam/cuda/CudaSfmValues.h \
  gtsam/slam/cuda/CudaSfmProjectionBatch.h \
  gtsam/slam/cuda/CudaSfmProjectionLinearization.h \
  gtsam/slam/cuda/CudaSfmProjectionLinearization.cu \
  gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h \
  gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu \
  gtsam/slam/tests/testCudaSfm.cpp \
  timing/timeSFMBAL.cpp
git commit -m "fix: stabilize CUDA BAL LM verification"
```

If verification required no code changes after Task 7, do not create an empty commit.

## Plan Self-Review

Spec coverage:

- Analytic production Jacobians: Task 3 and Task 4.
- Numeric Jacobians only for tests: Task 3 host numeric helper.
- `PinholeCamera<Cal3Bundler>` and `Point3`: Task 1 device types and packing.
- `GeneralSFMFactor` residual semantics: Task 1 measurement sign and Task 3 residual test.
- Full sparse normal equations: Task 4.
- cuDSS solve: Task 5.
- Unit noise: Task 3 residual and no noise-model upload.
- Explicit `--cuda-lm`: Task 7.
- Download optimized result: Task 1 `DownloadSfmValues` and Task 6 result.

Type consistency:

- Camera type: `DevicePinholeCameraCal3Bundler`.
- Camera type ID: `kDevicePinholeCameraCal3BundlerType`.
- Point type: `DevicePoint3`.
- Point type ID: `kDevicePoint3Type`.
- Projection linearization type: `CudaSfmProjectionLinearization`.
- LM API: `OptimizeCudaSfm`.

Implementation risk checkpoints:

- The analytic Jacobian must follow `CalibratedCamera.cpp`, `PinholeCamera.h`, and `Cal3Bundler.cpp`, not the old Snavely projection.
- The device retract must follow `Pose3::ChartAtOrigin::Retract`, which is `Pose3::Expmap` in this build.
- The current scratch test diff must be rewritten during Task 1 because it assumes `CudaCamera9` and OpenGL y-coordinate conversion.
