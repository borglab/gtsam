/**
 * @file  ImuProcessNoise.h
 * @brief Shared IMU process-noise PSD helpers for tg_eqf / tfg_inekf / mekf
 *        scenario runners.
 *
 * Runners build continuous-time PSDs (variances) from the same CLI sigmas,
 * optionally apply a shared numerical floor on ideal-IMU runs, and convert to
 * each filter's ImuNoise type. Mapping into the covariance (lift B-matrix vs
 * diagonal Qc) stays inside each filter.
 */
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace imu_scenarios {

/// Continuous-time IMU noise PSDs, per-axis (diagonal, not isotropic).
struct ImuNoisePSD {
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();      ///< gyro white-noise PSD      (rad^2/s)
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();     ///< accelerometer white-noise PSD (m^2/s^3)
  Eigen::Vector3d gyro_rw = Eigen::Vector3d::Zero();   ///< gyro bias random-walk PSD   (rad^2/s^3)
  Eigen::Vector3d accel_rw = Eigen::Vector3d::Zero();  ///< accel bias random-walk PSD  (m^2/s^5)
};

/// Build PSDs from continuous noise densities: PSD = sigma^2, per axis.
template <typename RunOptions>
inline ImuNoisePSD imuNoiseFromOptions(const RunOptions& opts) {
  ImuNoisePSD n;
  n.gyro = opts.gyro_noise_sigma.array().square();
  n.accel = opts.accel_noise_sigma.array().square();
  n.gyro_rw = opts.gyro_bias_rw.array().square();
  n.accel_rw = opts.accel_bias_rw.array().square();
  return n;
}

inline bool isZeroProcessNoise(const ImuNoisePSD& n) {
  return n.gyro.isZero() && n.accel.isZero() && n.gyro_rw.isZero() &&
         n.accel_rw.isZero();
}

/// Small continuous-time floor used when the simulated IMU is ideal (all CLI
/// sigmas zero) so covariance propagation stays numerically regularized.
inline ImuNoisePSD defaultProcessNoiseFloor() {
  ImuNoisePSD n;
  n.gyro = Eigen::Vector3d::Constant(1e-4);
  n.accel = Eigen::Vector3d::Constant(1e-3);
  n.gyro_rw = Eigen::Vector3d::Constant(1e-6);
  n.accel_rw = Eigen::Vector3d::Constant(1e-5);
  return n;
}

/// Per-axis: a channel's floor is applied to whichever of its own axes are
/// exactly zero, not to the whole 3-vector at once.
inline ImuNoisePSD applyProcessNoiseFloor(const ImuNoisePSD& psd,
                                          const ImuNoisePSD& floor) {
  auto floorAxes = [](const Eigen::Vector3d& v, const Eigen::Vector3d& f) {
    return Eigen::Vector3d(
        (v.array() == 0.0).select(f.array(), v.array()));
  };
  ImuNoisePSD out;
  out.gyro = floorAxes(psd.gyro, floor.gyro);
  out.accel = floorAxes(psd.accel, floor.accel);
  out.gyro_rw = floorAxes(psd.gyro_rw, floor.gyro_rw);
  out.accel_rw = floorAxes(psd.accel_rw, floor.accel_rw);
  return out;
}

/// PSDs from CLI; when every channel is unset, apply
/// defaultProcessNoiseFloor().
template <typename RunOptions>
inline ImuNoisePSD resolvedProcessNoise(const RunOptions& opts) {
  const ImuNoisePSD from_opts = imuNoiseFromOptions(opts);
  if (isZeroProcessNoise(from_opts)) {
    return defaultProcessNoiseFloor();
  }
  return from_opts;
}

/// Copy PSD fields into a filter-specific ImuNoise struct (identical layout).
template <typename FilterImuNoise>
inline FilterImuNoise toFilterImuNoise(const ImuNoisePSD& psd) {
  FilterImuNoise n;
  n.gyro = psd.gyro;
  n.accel = psd.accel;
  n.gyro_rw = psd.gyro_rw;
  n.accel_rw = psd.accel_rw;
  return n;
}

}  // namespace imu_scenarios
