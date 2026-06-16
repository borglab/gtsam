#pragma once

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <vector>

namespace gtsam::cuda {

inline CudaCamera9 PackCamera9(const SfmCamera& camera) {
  const Pose3 openGlPose = gtsam2openGL(camera.pose());
  const Vector3 r = Rot3::Logmap(openGlPose.rotation());
  const Point3& t = openGlPose.translation();
  const Cal3Bundler& calibration = camera.calibration();

  return {{r(0), r(1), r(2)},
          {t(0), t(1), t(2)},
          calibration.fx(),
          calibration.k1(),
          calibration.k2()};
}

inline CudaPoint3 PackPoint3(const Point3& point) {
  return {point(0), point(1), point(2)};
}

inline DeviceValues PackSfmValues(const SfmData& data,
                                  cudaStream_t stream = nullptr) {
  std::vector<Key> cameraKeys;
  std::vector<CudaCamera9> cameras;
  cameraKeys.reserve(data.numberCameras());
  cameras.reserve(data.numberCameras());
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    cameraKeys.push_back(symbol_shorthand::C(i));
    cameras.push_back(PackCamera9(data.camera(i)));
  }

  std::vector<Key> pointKeys;
  std::vector<CudaPoint3> points;
  pointKeys.reserve(data.numberTracks());
  points.reserve(data.numberTracks());
  for (size_t i = 0; i < data.numberTracks(); ++i) {
    pointKeys.push_back(symbol_shorthand::P(i));
    points.push_back(PackPoint3(data.track(i).point3()));
  }

  DeviceValues values;
  values.addBlock<CudaCamera9>(kCudaSfmCamera9Type, 9, cameraKeys, cameras,
                               stream);
  values.addBlock<CudaPoint3>(kCudaSfmPoint3Type, 3, pointKeys, points, stream);
  return values;
}

}  // namespace gtsam::cuda
