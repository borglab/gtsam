/**
 * @file  testInitialCovariance.cpp
 * @brief Monte Carlo check that each filter's initial covariance describes the
 *        initial error the scenario harness actually draws.
 *
 * sampleInitialEstimate() perturbs the truth by a physical navigation error
 * with covariance P0 (see PhysicalStateCovariance). Each scenario adapter hands
 * P0 to its filter's initialCovariance(X0, P0). If that conversion is right,
 * the error the filter reports at t = 0, whitened by the covariance it was
 * given, has identity covariance. The truth here is rotated and P0 is
 * anisotropic, so passing P0 through unchanged fails the same check.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam_unstable/examples/scenario_harness/ScenarioHarness.h>
#include <gtsam_unstable/navigation/MEKF.h>
#include <gtsam_unstable/navigation/TFGInEKF.h>
#include <gtsam_unstable/navigation/TGEqF.h>

#include <Eigen/Cholesky>
#include <functional>
#include <random>

using namespace gtsam;
using namespace imu_scenarios;

/* ************************************************************************* */
namespace fixture {

using Matrix15 = Eigen::Matrix<double, 15, 15>;
using Vector15 = Eigen::Matrix<double, 15, 1>;

// Sampling error over 20000 draws is about 0.01 per whitened entry, up to
// about 0.03 over all 120 entries. The conversion is first order: the group
// charts' J_l(dtheta)^{-1} factor leaks a rotated share of the larger position
// and velocity sigmas into the smaller axes, a further ~0.01 at these sigmas.
// Passing P0 through unchanged is off by about 6.
constexpr int kSamples = 20000;
constexpr double kTol = 0.06;
constexpr double kWrongChart = 0.5;

/// Anisotropic per-axis sigmas on every block. The attitude sigmas and the
/// spread of the position and velocity sigmas stay moderate, since the
/// second-order residual grows with sigma_theta times that spread.
RunOptions anisotropicOptions() {
  RunOptions opts;
  opts.init_att_sigma = Eigen::Vector3d(0.01, 0.02, 0.05);
  opts.init_vel_sigma = Eigen::Vector3d(0.3, 0.1, 0.2);
  opts.init_pos_sigma = Eigen::Vector3d(1.0, 0.5, 0.8);
  opts.init_gyro_bias_sigma = Eigen::Vector3d(0.01, 0.005, 0.02);
  opts.init_accel_bias_sigma = Eigen::Vector3d(0.1, 0.05, 0.2);
  return opts;
}

/// A rotated, moving truth. Its biases are zero, matching the zero-mean bias
/// draws, so every error component is zero-mean.
NavState rotatedTruth() {
  return NavState(Rot3::RzRyRx(0.3, -0.4, 1.9), Point3(5.0, 3.0, -2.0),
                  Velocity3(1.0, -0.7, 0.2));
}

/// One draw: the error the filter reports and the covariance it reports it
/// with, over the physical block.
struct Draw {
  Vector15 error;
  Matrix15 covariance;
};

/// Largest deviation from identity of the second moment of the errors, each
/// whitened by its own covariance.
double whitenedDeviation(
    const std::function<Draw(const InitialEstimate&)>& draw) {
  const RunOptions opts = anisotropicOptions();
  std::mt19937 rng(7);
  Matrix15 moment = Matrix15::Zero();
  for (int i = 0; i < kSamples; ++i) {
    const Draw d = draw(sampleInitialEstimate(rng, opts, rotatedTruth()));
    const Vector15 w = d.covariance.llt().matrixL().solve(d.error);
    moment += w * w.transpose() / kSamples;
  }
  return (moment - Matrix15::Identity()).cwiseAbs().maxCoeff();
}

}  // namespace fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace tfg_inekf {

using tfg::TFGInEKF;
using tfg::TwoFrameGroup;

// Builds the filter as TFGInEKFScenarioExample does, optionally skipping the
// conversion of P0.
fixture::Draw draw(const InitialEstimate& initial, bool convert) {
  const PhysicalStateCovariance P0 =
      initialCovariance(fixture::anisotropicOptions());
  const TwoFrameGroup X0 = TwoFrameGroup::FromState(
      initial.nav.attitude(), initial.nav.velocity(), initial.nav.position(),
      initial.bias_gyro, initial.bias_accel);
  const TFGInEKF filter(X0, convert ? TFGInEKF::initialCovariance(X0, P0) : P0);
  const NavState truth = fixture::rotatedTruth();
  const TwoFrameGroup X_true = TwoFrameGroup::FromState(
      truth.attitude(), truth.velocity(), truth.position(), Vector3::Zero(),
      Vector3::Zero());
  return {filter.errorStateVector(X_true), filter.covariance()};
}

// TFG-InEKF errors at t = 0 follow the converted covariance, not P0 itself.
TEST(InitialCovariance, TFGInEKFMatchesSampledError) {
  EXPECT(fixture::whitenedDeviation([](const InitialEstimate& initial) {
           return draw(initial, true);
         }) < fixture::kTol);
  EXPECT(fixture::whitenedDeviation([](const InitialEstimate& initial) {
           return draw(initial, false);
         }) > fixture::kWrongChart);
}

}  // namespace tfg_inekf
/* ************************************************************************* */

/* ************************************************************************* */
namespace mekf_filter {

using mekf::MEKF;
using mekf::MEKFState;

// Builds the filter as MEKFScenarioExample does, optionally skipping the
// conversion of P0.
fixture::Draw draw(const InitialEstimate& initial, bool convert) {
  const PhysicalStateCovariance P0 =
      initialCovariance(fixture::anisotropicOptions());
  const MEKFState X0(initial.nav.attitude(), initial.nav.velocity(),
                     initial.nav.position(), initial.bias_gyro,
                     initial.bias_accel);
  const MEKF filter(X0, convert ? MEKF::initialCovariance(X0, P0) : P0);
  const NavState truth = fixture::rotatedTruth();
  const MEKFState X_true(truth.attitude(), truth.velocity(), truth.position(),
                         Vector3::Zero(), Vector3::Zero());
  return {filter.errorStateVector(X_true), filter.covariance()};
}

// MEKF errors at t = 0 follow the converted covariance, not P0 itself: the
// body-frame attitude error sees the anisotropic attitude sigmas rotated.
TEST(InitialCovariance, MEKFMatchesSampledError) {
  EXPECT(fixture::whitenedDeviation([](const InitialEstimate& initial) {
           return draw(initial, true);
         }) < fixture::kTol);
  EXPECT(fixture::whitenedDeviation([](const InitialEstimate& initial) {
           return draw(initial, false);
         }) > fixture::kWrongChart);
}

}  // namespace mekf_filter
/* ************************************************************************* */

/* ************************************************************************* */
namespace tg_eqf {

using tgeqf::State;
using tgeqf::TGEqF;

State state(const NavState& nav, const Vector3& bias_gyro,
            const Vector3& bias_accel) {
  Eigen::Matrix<double, 3, 2> vp;
  vp.col(0) = nav.velocity();
  vp.col(1) = nav.position();
  State xi;
  xi.T = ExtendedPose3<2>(nav.attitude(), vp);
  xi.b.segment<3>(0) = bias_gyro;
  xi.b.segment<3>(3) = bias_accel;
  return xi;
}

// Builds the filter as TGEqFScenarioExample does, with the origin at the
// identity and the belief carried in the group element, optionally skipping
// the conversion of P0. The error and covariance are both in the origin chart,
// after the constructor's transport. Only the physical block is compared: the
// virtual bias has no physical error to check it against.
fixture::Draw draw(const InitialEstimate& initial, bool convert) {
  const PhysicalStateCovariance P0 =
      initialCovariance(fixture::anisotropicOptions());
  const State xi_hat0 =
      state(initial.nav, initial.bias_gyro, initial.bias_accel);
  TGEqF::Covariance18 Sigma0 = TGEqF::initialCovariance(xi_hat0, P0);
  if (!convert) {
    Sigma0.topLeftCorner<15, 15>() = P0;
    Sigma0.topRightCorner<15, 3>().setZero();
    Sigma0.bottomLeftCorner<3, 15>().setZero();
  }
  const TGEqF filter(tgeqf::phiInverse(State::identity(), xi_hat0), Sigma0);
  const State xi_true =
      state(fixture::rotatedTruth(), Vector3::Zero(), Vector3::Zero());
  return {filter.errorStateVector(xi_true).head<15>(),
          filter.errorCovariance().topLeftCorner<15, 15>()};
}

// TG-EqF errors at t = 0 follow the converted covariance, not P0 itself.
TEST(InitialCovariance, TGEqFMatchesSampledError) {
  EXPECT(fixture::whitenedDeviation([](const InitialEstimate& initial) {
           return draw(initial, true);
         }) < fixture::kTol);
  EXPECT(fixture::whitenedDeviation([](const InitialEstimate& initial) {
           return draw(initial, false);
         }) > fixture::kWrongChart);
}

}  // namespace tg_eqf
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
