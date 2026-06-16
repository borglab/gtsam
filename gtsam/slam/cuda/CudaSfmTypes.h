#pragma once

namespace gtsam::cuda {

inline constexpr unsigned int kCudaSfmCamera9Type = 0x53464d43u;
inline constexpr unsigned int kCudaSfmPoint3Type = 0x53464d50u;

struct CudaCamera9 {
  double r[3];
  double t[3];
  double f;
  double k1;
  double k2;
};

struct CudaPoint3 {
  double x;
  double y;
  double z;
};

struct CudaSfmObservation {
  int cameraSlot;
  int pointSlot;
  double measuredU;
  double measuredV;
};

}  // namespace gtsam::cuda
