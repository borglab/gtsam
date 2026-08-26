/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmValues.h
 * @brief   Packs host SFM cameras and landmarks into device values
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/internal/DeviceValues.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/sfm/cuda/internal/DeviceGeometryTypes.h>

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {

/// Converts one host SFM camera to its trivially copyable device form.
inline DevicePinholeCameraCal3Bundler packPinholeCameraCal3Bundler(
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

/// Converts one host point to its trivially copyable device form.
inline DevicePoint3 packDevicePoint3(const Point3& point) {
  return {point.x(), point.y(), point.z()};
}

/// Host construction, device allocation, and upload costs for SFM values.
struct SfmValuesPackProfile {
  double hostBuildElapsed = 0.0;
  double deviceAllocElapsed = 0.0;
  DeviceTransferSummary h2d;
};

/// Host allocation, download, and Values reconstruction costs.
struct SfmValuesDownloadProfile {
  double hostAllocElapsed = 0.0;
  double hostBuildElapsed = 0.0;
  DeviceTransferSummary d2h;
};

namespace internal {

inline double sfmValuesElapsedSince(
    std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       start)
      .count();
}

}  // namespace internal

/// Uploads SFM values using explicit camera and point keys.
inline DeviceValues packSfmValues(const SfmData& data,
                                  const std::vector<Key>& cameraKeys,
                                  const std::vector<Key>& pointKeys,
                                  cudaStream_t stream = nullptr,
                                  SfmValuesPackProfile* profile =
                                      nullptr) {
  if (profile) {
    *profile = SfmValuesPackProfile{};
  }
  if (cameraKeys.size() != data.numberCameras()) {
    throw std::invalid_argument(
        "packSfmValues camera key count does not match SfmData");
  }
  if (pointKeys.size() != data.numberTracks()) {
    throw std::invalid_argument(
        "packSfmValues point key count does not match SfmData");
  }

  const auto hostBuildStart =
      profile ? std::chrono::steady_clock::now()
              : std::chrono::steady_clock::time_point{};
  std::vector<DevicePinholeCameraCal3Bundler> cameras;
  cameras.reserve(data.numberCameras());
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    cameras.push_back(packPinholeCameraCal3Bundler(data.camera(i)));
  }

  std::vector<DevicePoint3> points;
  points.reserve(data.numberTracks());
  for (size_t i = 0; i < data.numberTracks(); ++i) {
    points.push_back(packDevicePoint3(data.track(i).point3()));
  }
  if (profile) {
    profile->hostBuildElapsed =
        internal::sfmValuesElapsedSince(hostBuildStart);
  }

  DeviceValues values;
  DeviceTransferTiming cameraUpload;
  DeviceTransferTiming pointUpload;
  double cameraDeltaAllocElapsed = 0.0;
  double pointDeltaAllocElapsed = 0.0;
  values.addBlock<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType,
      kDevicePinholeCameraCal3BundlerTangentDim, cameraKeys, cameras, stream,
      profile ? &cameraUpload : nullptr,
      profile ? &cameraDeltaAllocElapsed : nullptr);
  values.addBlock<DevicePoint3>(kDevicePoint3Type, kDevicePoint3TangentDim,
                                pointKeys, points, stream,
                                profile ? &pointUpload : nullptr,
                                profile ? &pointDeltaAllocElapsed : nullptr);
  if (profile) {
    profile->h2d.add(cameraUpload);
    profile->h2d.add(pointUpload);
    profile->deviceAllocElapsed =
        profile->h2d.resizeElapsed + cameraDeltaAllocElapsed +
        pointDeltaAllocElapsed;
  }
  return values;
}

/// Uploads SFM values using conventional C(i) and P(i) keys.
inline DeviceValues packSfmValues(const SfmData& data,
                                  cudaStream_t stream = nullptr,
                                  SfmValuesPackProfile* profile =
                                      nullptr) {
  std::vector<Key> cameraKeys;
  cameraKeys.reserve(data.numberCameras());
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    cameraKeys.push_back(symbol_shorthand::C(i));
  }

  std::vector<Key> pointKeys;
  pointKeys.reserve(data.numberTracks());
  for (size_t i = 0; i < data.numberTracks(); ++i) {
    pointKeys.push_back(symbol_shorthand::P(i));
  }

  return packSfmValues(data, cameraKeys, pointKeys, stream, profile);
}

/// Allocates uninitialized SFM device blocks matching a reference layout.
inline DeviceValues allocateSfmValuesLike(const DeviceValues& reference) {
  const auto& cameraBlock =
      reference.block<DevicePinholeCameraCal3Bundler>(
          kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = reference.block<DevicePoint3>(kDevicePoint3Type);

  DeviceValues values;
  values.addUninitializedBlock<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType, cameraBlock.tangentDim,
      cameraBlock.keys);
  values.addUninitializedBlock<DevicePoint3>(
      kDevicePoint3Type, pointBlock.tangentDim, pointBlock.keys);
  return values;
}

/// Downloads device SFM values into a host Values container.
inline Values downloadSfmValues(const DeviceValues& deviceValues,
                                cudaStream_t stream = nullptr,
                                SfmValuesDownloadProfile* profile =
                                    nullptr) {
  if (profile) {
    *profile = SfmValuesDownloadProfile{};
  }
  std::vector<DevicePinholeCameraCal3Bundler> cameras;
  std::vector<DevicePoint3> points;
  const auto& cameraBlock =
      deviceValues.block<DevicePinholeCameraCal3Bundler>(
          kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = deviceValues.block<DevicePoint3>(kDevicePoint3Type);
  if (profile) {
    const DeviceTransferTiming cameraDownload =
        cameraBlock.values.downloadProfiled(&cameras, stream);
    const DeviceTransferTiming pointDownload =
        pointBlock.values.downloadProfiled(&points, stream);
    profile->d2h.add(cameraDownload);
    profile->d2h.add(pointDownload);
    profile->hostAllocElapsed = profile->d2h.resizeElapsed;
  } else {
    cameraBlock.values.download(&cameras, stream);
    pointBlock.values.download(&points, stream);
  }
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));

  const auto hostBuildStart =
      profile ? std::chrono::steady_clock::now()
              : std::chrono::steady_clock::time_point{};
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
    const Cal3Bundler calibration(cameras[i].f, cameras[i].k1, cameras[i].k2);
    result.insert(cameraBlock.keys[i], SfmCamera(pose, calibration));
  }
  for (size_t i = 0; i < points.size(); ++i) {
    result.insert(pointBlock.keys[i],
                  Point3(points[i].x, points[i].y, points[i].z));
  }
  if (profile) {
    profile->hostBuildElapsed =
        internal::sfmValuesElapsedSince(hostBuildStart);
  }
  return result;
}

}  // namespace gtsam::cuda
