#pragma once

namespace gtsam::cuda {

struct CudaSfmObservation {
  int cameraSlot;
  int pointSlot;
  double measuredU;
  double measuredV;
};

}  // namespace gtsam::cuda
