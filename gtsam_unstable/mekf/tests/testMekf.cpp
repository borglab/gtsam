/**
 * @file  testMekf.cpp
 * @brief Integration tests for the MultiplicativeEKF wrapper (predict +
 * update).
 *
 * Tangent block order: see ../README.md.
 * Position block lives at indices 6..8.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/mekf/MEKF.h>

using namespace mekf;
using namespace gtsam;

static constexpr double kTol = 1e-9;

// The filter is frame-agnostic -- gravity is a propagate() argument -- so these
// tests pick a convention of their own: Z-up (ENU), gravity along -z.
static const Eigen::Vector3d kTestGravity(0.0, 0.0, -9.81);

using Cov15 = Eigen::Matrix<double, 15, 15>;
using Cov3 = Eigen::Matrix<double, 3, 3>;

static bool veq(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                double tol = kTol) {
  return assert_equal((Vector)a, (Vector)b, tol);
}

// Min eigenvalue of a symmetric matrix (PSD check).
static double minEig(const Eigen::MatrixXd& M) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(M);
  return es.eigenvalues().minCoeff();
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST(Mekf, ConstructorStoresStateAndCovariance) {
  MekfState X0(Rot3::Rz(0.2), Eigen::Vector3d(0.1, 0, 0),
               Eigen::Vector3d(1, 2, 3), Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 0.5;
  MultiplicativeEKF ekf(X0, P0);

  EXPECT(traits<MekfState>::Equals(ekf.state(), X0, kTol));
  EXPECT(assert_equal((Matrix)P0, (Matrix)ekf.covariance(), kTol));
  EXPECT(veq(ekf.position(), Eigen::Vector3d(1, 2, 3)));
  EXPECT(ekf.attitude().equals(Rot3::Rz(0.2), kTol));
}

// ---------------------------------------------------------------------------
// Predict
// ---------------------------------------------------------------------------

TEST(Mekf, PredictKeepsCovarianceSymmetricPSD) {
  MultiplicativeEKF ekf(MekfState::identity(), Cov15::Identity() * 0.1);

  Cov15 Qc = Cov15::Identity() * 1e-3;
  ekf.propagate(Eigen::Vector3d(0.05, -0.02, 0.1),
                Eigen::Vector3d(0.0, 0.0, 9.81), kTestGravity, Qc, 0.01);

  const Cov15 P = ekf.covariance();
  EXPECT(assert_equal((Matrix)P, (Matrix)P.transpose(), 1e-12));  // symmetric
  EXPECT(minEig(P) > 0.0);                                        // PSD
}

// Mean prediction matches the world-frame strapdown step (Eq. 3).
TEST(Mekf, PredictMeanFollowsDynamics) {
  const Rot3 R0 = Rot3::Rz(0.3);
  const Eigen::Vector3d v0(0.2, -0.1, 0.0), p0(1.0, 0.0, -1.0);
  MultiplicativeEKF ekf(
      MekfState(R0, v0, p0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      Cov15::Identity() * 0.01);

  const Eigen::Vector3d omega(0.0, 0.0, 0.1), accel(0.0, 0.0, 0.0);
  const double dt = 1e-4;
  ekf.propagate(omega, accel, kTestGravity, Cov15::Identity() * 1e-6, dt);

  const Eigen::Vector3d g = kTestGravity;
  const Rot3 R1 = R0 * Rot3::Expmap(omega * dt);
  const Eigen::Vector3d v1 = v0 + (R0 * accel + g) * dt;
  const Eigen::Vector3d p1 = p0 + v0 * dt;

  EXPECT(ekf.attitude().equals(R1, 1e-9));
  EXPECT(veq(ekf.velocity(), v1, 1e-7));
  EXPECT(veq(ekf.position(), p1, 1e-7));
}

// Covariance grows when propagating with process noise and no measurement.
TEST(Mekf, PredictInflatesCovariance) {
  MultiplicativeEKF ekf(MekfState::identity(), Cov15::Identity() * 0.1);
  const double trace_before = ekf.covariance().trace();
  ekf.propagate(Eigen::Vector3d(0.01, 0.02, 0.03),
                Eigen::Vector3d(0.0, 0.0, 9.81), kTestGravity,
                Cov15::Identity() * 1e-2, 0.01);
  EXPECT(ekf.covariance().trace() > trace_before);
}

// ImuNoise overload must match propagate with the equivalent diagonal Qc.
TEST(Mekf, PropagateImuNoiseMatchesDiagonalQc) {
  ImuNoise nz;
  nz.gyro = Eigen::Vector3d::Constant(1e-4);
  nz.accel = Eigen::Vector3d::Constant(1e-3);
  nz.gyro_rw = Eigen::Vector3d::Constant(1e-6);
  nz.accel_rw = Eigen::Vector3d::Constant(1e-5);
  const Eigen::Vector3d omega(0.05, -0.02, 0.01);
  const Eigen::Vector3d accel(0.1, -0.2, 9.85);
  const double dt = 0.01;

  MultiplicativeEKF f1(MekfState::identity(), Cov15::Identity() * 0.1);
  MultiplicativeEKF f2(MekfState::identity(), Cov15::Identity() * 0.1);

  Cov15 Qc = Cov15::Zero();
  Qc.block<3, 3>(0, 0) = nz.gyro.asDiagonal();
  Qc.block<3, 3>(3, 3) = nz.accel.asDiagonal();
  Qc.block<3, 3>(9, 9) = nz.gyro_rw.asDiagonal();
  Qc.block<3, 3>(12, 12) = nz.accel_rw.asDiagonal();

  f1.propagate(omega, accel, kTestGravity, nz, dt);
  f2.propagate(omega, accel, kTestGravity, Qc, dt);

  EXPECT(traits<MekfState>::Equals(f1.state(), f2.state(), 1e-12));
  EXPECT(assert_equal((Matrix)f1.covariance(), (Matrix)f2.covariance(), 1e-9));
}

// ---------------------------------------------------------------------------
// Position update
// ---------------------------------------------------------------------------

// A position measurement reduces the position-block covariance.
TEST(Mekf, UpdateReducesPositionCovariance) {
  Cov15 P0 = Cov15::Identity() * 0.1;
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 4.0;  // large position uncertainty
  MultiplicativeEKF ekf(MekfState::identity(), P0);

  const double trace_before = ekf.covariance().block<3, 3>(6, 6).trace();
  ekf.update_position(Eigen::Vector3d(0.5, -0.3, 0.2), Cov3::Identity() * 0.01);
  const double trace_after = ekf.covariance().block<3, 3>(6, 6).trace();

  EXPECT(trace_after < trace_before);
  EXPECT(minEig(ekf.covariance()) > 0.0);
}

// A single accurate update pulls a wrong position estimate toward the fix.
TEST(Mekf, UpdateCorrectsPosition) {
  MekfState X0(Rot3::Identity(), Eigen::Vector3d::Zero(),
               Eigen::Vector3d(1.0, -1.0, 0.5),  // wrong position
               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 1e-4;
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 1.0;  // trust measurement more
  MultiplicativeEKF ekf(X0, P0);

  const Eigen::Vector3d truth(0.0, 0.0, 0.0);
  ekf.update_position(truth, Cov3::Identity() * 1e-4);

  // Estimate should move most of the way to the measurement.
  const double err_before = (Eigen::Vector3d(1.0, -1.0, 0.5) - truth).norm();
  const double err_after = (ekf.position() - truth).norm();
  EXPECT(err_after < 0.05 * err_before);
}

// ---------------------------------------------------------------------------
// DVL update
// ---------------------------------------------------------------------------

// A DVL measurement reduces the velocity-block covariance.
TEST(Mekf, UpdateDvlReducesVelocityCovariance) {
  Cov15 P0 = Cov15::Identity() * 0.1;
  P0.block<3, 3>(3, 3) = Cov3::Identity() * 4.0;  // large velocity uncertainty
  MultiplicativeEKF ekf(MekfState::identity(), P0);

  const double trace_before = ekf.covariance().block<3, 3>(3, 3).trace();
  ekf.update_dvl(Eigen::Vector3d(0.3, -0.1, 0.0), Cov3::Identity() * 0.01);
  const double trace_after = ekf.covariance().block<3, 3>(3, 3).trace();

  EXPECT(trace_after < trace_before);
  EXPECT(minEig(ekf.covariance()) > 0.0);
}

// A single accurate DVL update pulls a wrong velocity estimate toward truth.
TEST(Mekf, UpdateDvlCorrectsVelocity) {
  const Rot3 R0 = Rot3::Rz(0.5);
  const Eigen::Vector3d truth_v = R0 * Eigen::Vector3d(0.2, -0.1, 0.05);
  MekfState X0(R0, Eigen::Vector3d(1.0, 1.0, -1.0),  // wrong world velocity
               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 1e-4;
  P0.block<3, 3>(3, 3) = Cov3::Identity() * 1.0;  // trust the DVL more
  MultiplicativeEKF ekf(X0, P0);

  // DVL reports the body-frame velocity consistent with truth_v: R0^T truth_v.
  const Eigen::Vector3d z_dvl = R0.unrotate(truth_v);
  ekf.update_dvl(z_dvl, Cov3::Identity() * 1e-4);

  const double err_before = (Eigen::Vector3d(1.0, 1.0, -1.0) - truth_v).norm();
  const double err_after = (ekf.velocity() - truth_v).norm();
  EXPECT(err_after < 0.05 * err_before);
}

/* ************************************************************************* */
namespace depth_update {

using Cov1 = MultiplicativeEKF::Covariance1;

// 1x1 depth noise matrix from a stddev in metres.
Cov1 depthNoise(double sigma) {
  Cov1 R;
  R(0, 0) = sigma * sigma;
  return R;
}

MekfState offsetState() {
  return MekfState(Rot3::Rx(M_PI / 6), Eigen::Vector3d(0.5, -0.2, 0.1),
                   Eigen::Vector3d(1.0, 2.0, -3.0), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero());
}

// A single depth update pulls the vertical estimate most of the way to the
// measurement when the depth noise is much tighter than the prior.
TEST(Mekf, DepthUpdateCorrectsVerticalPosition) {
  MultiplicativeEKF ekf(MekfState::identity(), Cov15::Identity() * 0.01);

  const double z_depth = -0.8;
  ekf.update_depth(z_depth, depthNoise(0.01));

  EXPECT(std::abs(ekf.position().z() - z_depth) < 0.1);
}

// The inflated horizontal variance keeps a depth update off the x and y axes
// when the prior is uncorrelated.
TEST(Mekf, DepthUpdateLeavesHorizontalNearlyUnchanged) {
  MultiplicativeEKF ekf(offsetState(), Cov15::Identity() * 0.01);

  const Eigen::Vector3d before = ekf.position();
  ekf.update_depth(before.z() - 0.8, depthNoise(0.01));
  const Eigen::Vector3d delta = ekf.position() - before;

  EXPECT(std::abs(delta.z()) > 0.5);
  EXPECT(std::abs(delta.x()) < 0.02);
  EXPECT(std::abs(delta.y()) < 0.02);
}

// Only the vertical position covariance is informed; the horizontal blocks stay
// essentially where they were.
TEST(Mekf, DepthUpdateShrinksVerticalCovariance) {
  MultiplicativeEKF ekf(MekfState::identity(), Cov15::Identity() * 0.01);

  const Cov15 before = ekf.covariance();
  ekf.update_depth(-0.8, depthNoise(0.01));
  const Cov15 after = ekf.covariance();

  const double drop_z = before(8, 8) - after(8, 8);
  EXPECT(drop_z > 0.0);
  EXPECT(std::abs(before(6, 6) - after(6, 6)) < 0.1 * drop_z);
  EXPECT(std::abs(before(7, 7) - after(7, 7)) < 0.1 * drop_z);
  EXPECT(minEig(after) > 0.0);
}

// update_depth is exactly the pseudo-position update it documents.
TEST(Mekf, DepthUpdateMatchesEquivalentPositionUpdate) {
  MultiplicativeEKF via_depth(offsetState(), Cov15::Identity() * 0.01);
  MultiplicativeEKF via_position(offsetState(), Cov15::Identity() * 0.01);

  const double z_depth = -2.0;
  const double sigma_z = 0.02;
  via_depth.update_depth(z_depth, depthNoise(sigma_z));

  Cov3 R_pseudo = Cov3::Zero();
  R_pseudo(0, 0) = MultiplicativeEKF::kDefaultHorizontalVariance;
  R_pseudo(1, 1) = MultiplicativeEKF::kDefaultHorizontalVariance;
  R_pseudo(2, 2) = sigma_z * sigma_z;
  Eigen::Vector3d pseudo = via_position.position();
  pseudo.z() = z_depth;
  via_position.update_position(pseudo, R_pseudo);

  EXPECT(traits<MekfState>::Equals(via_position.state(), via_depth.state(),
                                   1e-12));
  EXPECT(assert_equal((Matrix)via_position.covariance(),
                      (Matrix)via_depth.covariance(), 1e-12));
}

// The horizontal-variance argument is live: shrinking it changes the correction
// a correlated prior produces, so the default is not silently ignored.
TEST(Mekf, DepthUpdateHorizontalVarianceIsRespected) {
  // Correlate p_x with p_z; the block [[0.01, 0.008], [0.008, 0.01]] is SPD.
  Cov15 P0 = Cov15::Identity() * 0.01;
  P0(6, 8) = P0(8, 6) = 0.008;

  MultiplicativeEKF inflated(MekfState::identity(), P0);
  MultiplicativeEKF pinned(MekfState::identity(), P0);

  const double z_depth = -0.8;
  inflated.update_depth(z_depth, depthNoise(0.01));
  pinned.update_depth(z_depth, depthNoise(0.01), /*horizontal_variance=*/1e-6);

  EXPECT((inflated.position() - pinned.position()).norm() > 1e-3);
}

}  // namespace depth_update
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
