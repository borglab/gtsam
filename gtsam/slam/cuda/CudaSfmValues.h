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
  const auto& pointBlock = deviceValues.block<DevicePoint3>(kDevicePoint3Type);
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
    const Cal3Bundler calibration(cameras[i].f, cameras[i].k1, cameras[i].k2);
    result.insert(cameraBlock.keys[i], SfmCamera(pose, calibration));
  }
  for (size_t i = 0; i < points.size(); ++i) {
    result.insert(pointBlock.keys[i],
                  Point3(points[i].x, points[i].y, points[i].z));
  }
  return result;
}

}  // namespace gtsam::cuda
