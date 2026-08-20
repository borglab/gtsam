#pragma once

#include <cstdint>

namespace gtsam::cuda {

inline constexpr uint32_t kDevicePinholeCameraCal3BundlerType = 0x50434243u;
inline constexpr uint32_t kDevicePoint3Type = 0x50544e33u;
inline constexpr int kDevicePinholeCameraCal3BundlerTangentDim = 9;
inline constexpr int kDevicePoint3TangentDim = 3;

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
