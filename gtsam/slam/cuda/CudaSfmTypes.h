#pragma once

namespace gtsam::cuda {

struct CudaSfmObservation {
  int cameraSlot;
  int pointSlot;
  double measuredU;
  double measuredV;
};

struct CudaSfmSqrtInfo2 {
  double r00;
  double r01;
  double r11;
};

enum class CudaSfmRobustModelKind {
  None,
  Huber,
  Tukey,
};

enum class CudaSfmRobustReweightScheme {
  Scalar,
  Block,
};

struct CudaSfmRobustModel {
  CudaSfmRobustModelKind kind;
  CudaSfmRobustReweightScheme reweightScheme;
  double parameter;
};

}  // namespace gtsam::cuda
