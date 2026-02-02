/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    imuFactorTesting.h
 * @brief   Common testing infrastructure
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <iostream>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

namespace {
[[maybe_unused]] static const Vector3 kZero = Z_3x1;
typedef imuBias::ConstantBias Bias;
[[maybe_unused]] static const Bias kZeroBiasHat, kZeroBias;

[[maybe_unused]] static const Vector3 kZeroOmegaCoriolis(0, 0, 0);
[[maybe_unused]] static const Vector3 kNonZeroOmegaCoriolis(0, 0.1, 0.1);

[[maybe_unused]] static const double kGravity = 10;
[[maybe_unused]] static const Vector3 kGravityAlongNavZDown(0, 0, kGravity);

// Realistic MEMS white noise characteristics. Angular and velocity random walk
// expressed in degrees respectively m/s per sqrt(hr).
auto radians = [](double t) { return t * M_PI / 180; };
[[maybe_unused]] static const double kGyroSigma =
    radians(0.5) / 60;                                        // 0.5 degree ARW
[[maybe_unused]] static const double kAccelSigma = 0.1 / 60;  // 10 cm VRW

// --------------------------- small helpers ---------------------------
// Rotation near: angle of R0^{-1} R1 should be <= tol_rad.
#define EXPECT_ROT3_NEAR(R0, R1, tol_rad)                                         \
  do {                                                                            \
    const gtsam::Vector3 _w = gtsam::Rot3::Logmap((R0).between((R1)));            \
    const double _w_norm = _w.norm();                                             \
    if (_w_norm > (tol_rad)) {                                                    \
      std::cout << "EXPECT_ROT3_NEAR failed\n"                                    \
                << "  w: [" << _w.transpose() << "]\n"                            \
                << "  w_norm: " << _w_norm << " tol: " << (tol_rad) << "\n"       \
                << "  R0:\n" << (R0).matrix() << "\n"                             \
                << "  R1:\n" << (R1).matrix() << "\n";                            \
    }                                                                             \
    EXPECT(_w_norm <= (tol_rad));                                                 \
  } while (0)

// Robust-ish numeric compare for matrices, with abs+rel tolerance on Fro norm.
// Passes if ||A-B||_F <= abs_tol + rel_tol * max(1, ||B||_F)
#define EXPECT_MAT_NEAR(A, B, abs_tol, rel_tol)                                   \
  do {                                                                            \
    const auto& _A = (A);                                                         \
    const auto& _B = (B);                                                         \
    const double _bnorm = _B.norm();                                              \
    const double _denom = std::max(1.0, _bnorm);                                  \
    const double _err   = (_A - _B).norm();                                       \
    const double _tol = (abs_tol) + (rel_tol) * _denom;                           \
    if (_err > _tol) {                                                            \
      std::cout << "EXPECT_MAT_NEAR failed\n"                                     \
                << "  err: " << _err << " tol: " << _tol                          \
                << " (abs: " << (abs_tol) << ", rel: " << (rel_tol) << ", bnorm: "\
                << _bnorm << ")\n"                                                \
                << "  A:\n" << _A << "\n"                                         \
                << "  B:\n" << _B << "\n";                                        \
    }                                                                             \
    EXPECT(_err <= _tol);                                                         \
  } while (0)

struct ImuSimConfig {
  double g = 9.81;
  double dt = 0.005;

  // Noise model used inside preintegration/ekf (white noise)
  double sigma_g_c = 5e-2;
  double sigma_a_c = 3e-2;
  double sigma_gw_c = 7e-3;
  double sigma_aw_c = 2e-3;

  gtsam::Rot3  Rws0;
  gtsam::Point3 pws0;
  gtsam::Vector3 vws0;
  gtsam::imuBias::ConstantBias bias;  // (acc, gyro)

  // NAV9 covariance (order: [dtheta, dp, dv])
  Eigen::Matrix<double,9,9> P0_nav9;

  explicit ImuSimConfig(unsigned int seed = 0, bool zero_bw = true) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> N01(0.0, 1.0);
    auto n = [&](){ return N01(rng); };

    const gtsam::Vector3 w0(0.2*n(), 0.2*n(), 0.2*n());
    Rws0 = gtsam::Rot3::Expmap(w0);

    pws0 = gtsam::Point3(n(), n(), n());
    vws0 = gtsam::Vector3(n(), n(), n());

    const gtsam::Vector3 ba0(0.05*n(), 0.05*n(), 0.05*n());
    const gtsam::Vector3 bg0(0.01*n(), 0.01*n(), 0.01*n());
    bias = gtsam::imuBias::ConstantBias(ba0, bg0);

    P0_nav9.setZero();
    P0_nav9.diagonal().segment<3>(0).setConstant(1.0);     // rot
    P0_nav9.diagonal().segment<3>(3).setConstant(100.0);   // pos
    P0_nav9.diagonal().segment<3>(6).setConstant(10.0);    // vel
    if (zero_bw) {
      sigma_gw_c = 0;
      sigma_aw_c = 0;
    }
  }
};

inline std::shared_ptr<gtsam::PreintegrationParams> MakeParamsU(const ImuSimConfig& cfg) {
  auto p = gtsam::PreintegrationParams::MakeSharedU(cfg.g);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  p->gyroscopeCovariance     = (cfg.sigma_g_c * cfg.sigma_g_c) * I;
  p->accelerometerCovariance = (cfg.sigma_a_c * cfg.sigma_a_c) * I;
  p->integrationCovariance   = 1e-12 * I;
  p->use2ndOrderCoriolis     = false;
  return p;
}

struct ImuRawSample {
  gtsam::Vector3 measuredOmega;
  gtsam::Vector3 measuredAcc;
  double dt;
};

inline std::vector<ImuRawSample> MakeRandomImuMeasurements(
    const ImuSimConfig& cfg,
    double T,
    unsigned int seed) {
  const int N = static_cast<int>(std::round(T / cfg.dt));
  std::vector<ImuRawSample> out;
  if (N <= 0) return out;
  out.reserve(N);

  const Vector3 acc_mean = Vector3(0.0, 0.0, cfg.g);
  for (int k = 0; k < N; ++k) {
    Eigen::Vector3d omega = Eigen::Vector3d::Random();  // range from [-1, 1]
    Eigen::Vector3d acc = Eigen::Vector3d::Random();
    acc += acc_mean;
    out.push_back(ImuRawSample{omega, acc, cfg.dt});
  }
  return out;
}
}  // namespace

namespace testing {

[[maybe_unused]] static std::shared_ptr<PreintegrationParams> Params() {
  auto p = PreintegrationParams::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  return p;
}

struct ImuMeasurement {
  ImuMeasurement(const Vector3& acc, const Vector3& gyro, double dt)
      : acc(acc), gyro(gyro), dt(dt) {}
  const Vector3 acc, gyro;
  const double dt;
};

template <typename PIM>
void integrateMeasurements(const vector<ImuMeasurement>& measurements,
                           PIM* pim) {
  for (const auto& m : measurements)
    pim->integrateMeasurement(m.acc, m.gyro, m.dt);
}

struct SomeMeasurements : vector<ImuMeasurement> {
  SomeMeasurements() {
    reserve(102);
    const double dt = 0.01, pi100 = M_PI / 100;
    emplace_back(Vector3(0.1, 0, 0), Vector3(pi100, 0, 0), dt);
    emplace_back(Vector3(0.1, 0, 0), Vector3(pi100, 0, 0), dt);
    for (int i = 1; i < 100; i++) {
      emplace_back(Vector3(0.05, 0.09, 0.01),
                   Vector3(pi100, pi100 * 3, 2 * pi100), dt);
    }
  }
};

}  // namespace testing
namespace {
// Macro to test ImuFactor with both Manifold and Tangent preintegration
// In the tests below the selected PreintegratedImuMeasurementsT is available
// as `PIM`, and the combined version as `CombinedPIM`.
#define TEST_PIM(testGroup, testName)                          \
  template <class PIM, class CombinedPIM>                      \
  void testGroup##testName##Helper(TestResult& result_,        \
                                   const std::string& name_);  \
  TEST(testGroup, testName) {                                  \
    using M = ManifoldPreintegration;                          \
    using PM = PreintegratedImuMeasurementsT<M>;               \
    using CM = PreintegratedCombinedMeasurementsT<M>;          \
    using T = TangentPreintegration;                           \
    using PT = PreintegratedImuMeasurementsT<T>;               \
    using CT = PreintegratedCombinedMeasurementsT<T>;          \
    testGroup##testName##Helper<PM, CM>(result_, this->name_); \
    testGroup##testName##Helper<PT, CT>(result_, this->name_); \
  }                                                            \
  template <class PIM, class CombinedPIM>                      \
  void testGroup##testName##Helper(TestResult& result_,        \
                                   const std::string& name_)
}  // namespace
