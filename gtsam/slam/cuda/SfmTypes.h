#pragma once

namespace gtsam::cuda {

struct SfmObservation {
  int cameraSlot;
  int pointSlot;
  double measuredU;
  double measuredV;
};

struct SfmSqrtInfo2 {
  double r00;
  double r01;
  double r11;
};

enum class SfmRobustModelKind {
  None,
  Huber,
  Tukey,
};

enum class SfmRobustReweightScheme {
  Scalar,
  Block,
};

struct SfmRobustModel {
  SfmRobustModelKind kind;
  SfmRobustReweightScheme reweightScheme;
  double parameter;
};

}  // namespace gtsam::cuda
