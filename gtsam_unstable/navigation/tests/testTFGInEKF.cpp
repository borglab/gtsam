/**
 * @file  testTFGInEKF.cpp
 * @brief Integration tests for the TFG-IEKF wrapper (predict + position
 * update).
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam_unstable/navigation/TFGInEKF.h>

#include <functional>

using namespace gtsam::tfg;
using namespace gtsam;

static constexpr double kTol = 1e-9;

// The filter is frame-agnostic -- gravity is a propagate() argument -- so these
// tests pick a convention of their own: Z-up (ENU), gravity along -z.
static Eigen::Vector3d testGravity() {
  return Eigen::Vector3d(0.0, 0.0, -9.81);
}

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

TEST(TFGInEKF, ConstructorStoresStateAndCovariance) {
  auto X0 = TwoFrameGroup::FromState(
      Rot3::Rz(0.2), Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(1, 2, 3),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 0.5;
  TFGInEKF ekf(X0, P0);

  EXPECT(traits<TwoFrameGroup>::Equals(ekf.state(), X0, kTol));
  EXPECT(assert_equal((Matrix)P0, (Matrix)ekf.covariance(), kTol));
  EXPECT(veq(ekf.position(), Eigen::Vector3d(1, 2, 3), kTol));
}

// ---------------------------------------------------------------------------
// Predict: covariance stays symmetric PSD; state mean follows the dynamics.
// ---------------------------------------------------------------------------

TEST(TFGInEKF, PredictKeepsCovarianceSymmetricPSD) {
  auto X0 = TwoFrameGroup::Identity();
  Cov15 P0 = Cov15::Identity() * 0.1;
  TFGInEKF ekf(X0, P0);

  Cov15 Qc = Cov15::Identity() * 1e-3;
  ekf.propagate(Eigen::Vector3d(0.05, -0.02, 0.1),
                Eigen::Vector3d(0.0, 0.0, 9.81), testGravity(), Qc, 0.01);

  const Cov15 P = ekf.covariance();
  EXPECT(assert_equal((Matrix)P, (Matrix)P.transpose(), 1e-12));  // symmetric
  EXPECT(minEig(P) > 0.0);                                        // PSD
}

// Mean prediction matches the world-frame dynamics (Eq. 3) over one step.
TEST(TFGInEKF, PredictMeanFollowsDynamics) {
  Rot3 R0 = Rot3::Rz(0.3);
  Eigen::Vector3d v0(0.2, -0.1, 0.0), p0(1.0, 0.0, -1.0);
  auto X0 = TwoFrameGroup::FromState(R0, v0, p0, Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero());
  TFGInEKF ekf(X0, Cov15::Identity() * 0.01);

  const Eigen::Vector3d omega(0.0, 0.0, 0.1), accel(0.0, 0.0, 0.0);
  const double dt = 1e-4;
  ekf.propagate(omega, accel, testGravity(), Cov15::Identity() * 1e-6, dt);

  const Eigen::Vector3d g = testGravity();
  Rot3 R1 = R0 * Rot3::Expmap(omega * dt);
  Eigen::Vector3d v1 = v0 + (R0 * accel + g) * dt;
  Eigen::Vector3d p1 = p0 + v0 * dt;

  EXPECT(ekf.attitude().equals(R1, 1e-9));
  EXPECT(veq(ekf.velocity(), v1, 1e-7));
  EXPECT(veq(ekf.position(), p1, 1e-7));
}

// ---------------------------------------------------------------------------
// Position update
// ---------------------------------------------------------------------------

// A position measurement reduces the position-block covariance.
TEST(TFGInEKF, UpdateReducesPositionCovariance) {
  auto X0 = TwoFrameGroup::Identity();
  Cov15 P0 = Cov15::Identity() * 0.1;
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 4.0;  // large position uncertainty
  TFGInEKF ekf(X0, P0);

  const double trace_before = ekf.covariance().block<3, 3>(6, 6).trace();
  ekf.updatePosition(Eigen::Vector3d(0.5, -0.3, 0.2), Cov3::Identity() * 0.01);
  const double trace_after = ekf.covariance().block<3, 3>(6, 6).trace();

  EXPECT(trace_after < trace_before);
  EXPECT(minEig(ekf.covariance()) > 0.0);
}

// A single accurate update pulls a wrong position estimate toward the
// measurement (here R = I so the position block is directly observed).
TEST(TFGInEKF, UpdateCorrectsPosition) {
  auto X0 = TwoFrameGroup::FromState(
      Rot3::Identity(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, -1.0, 0.5),  // wrong position
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 1e-4;
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 1.0;  // trust measurement more
  TFGInEKF ekf(X0, P0);

  const Eigen::Vector3d truth(0.0, 0.0, 0.0);
  ekf.updatePosition(truth, Cov3::Identity() * 1e-4);

  // Estimate should move substantially toward truth.
  EXPECT(ekf.position().norm() < 0.1);
}

// ---------------------------------------------------------------------------
// DVL update (body-frame velocity)
// ---------------------------------------------------------------------------

// A DVL measurement reduces the velocity-block covariance.
TEST(TFGInEKF, UpdateDvlReducesVelocityCovariance) {
  auto X0 = TwoFrameGroup::Identity();
  Cov15 P0 = Cov15::Identity() * 0.1;
  P0.block<3, 3>(3, 3) = Cov3::Identity() * 4.0;  // large velocity uncertainty
  TFGInEKF ekf(X0, P0);

  const double trace_before = ekf.covariance().block<3, 3>(3, 3).trace();
  ekf.updateDvl(Eigen::Vector3d(0.3, -0.1, 0.0), Cov3::Identity() * 0.01);
  const double trace_after = ekf.covariance().block<3, 3>(3, 3).trace();

  EXPECT(trace_after < trace_before);
  EXPECT(minEig(ekf.covariance()) > 0.0);
}

// A single accurate DVL update pulls a wrong velocity estimate toward truth.
// The DVL reports the body-frame velocity consistent with truth_v: R^T truth_v.
TEST(TFGInEKF, UpdateDvlCorrectsVelocity) {
  const Rot3 R0 = Rot3::Rz(0.5);
  const Eigen::Vector3d truth_v = R0 * Eigen::Vector3d(0.2, -0.1, 0.05);
  auto X0 = TwoFrameGroup::FromState(
      R0, Eigen::Vector3d(1.0, 1.0, -1.0),  // wrong world velocity
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 1e-4;
  P0.block<3, 3>(3, 3) = Cov3::Identity() * 1.0;  // trust the DVL more
  TFGInEKF ekf(X0, P0);

  const Eigen::Vector3d z_dvl = R0.unrotate(truth_v);
  ekf.updateDvl(z_dvl, Cov3::Identity() * 1e-4);

  const double err_before = (Eigen::Vector3d(1.0, 1.0, -1.0) - truth_v).norm();
  const double err_after = (ekf.velocity() - truth_v).norm();
  EXPECT(err_after < 0.05 * err_before);
}

// ---------------------------------------------------------------------------
// End-to-end: static vehicle, IMU + GNSS, filter converges to truth.
// ---------------------------------------------------------------------------

// Truth: static at origin, level. Filter starts with confident (level) attitude
// but uncertain position/velocity, fed IMU (100 Hz) + GNSS (10 Hz). End-to-end
// the filter must drive the error down and shrink the covariance.
//
// Note: in a purely static scene horizontal position and tilt are weakly
// distinguishable through gravity, and the first-order output Jacobian C0
// couples them (the y_hat^ block). So horizontal position converges to a small
// residual rather than exactly zero — tightened by the equivariant C* (B.35);
// see CstarConvergesFromLargeError below. We therefore assert strong error
// reduction, clean velocity convergence, and covariance shrink.
TEST(TFGInEKF, StaticConvergence) {
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  const Eigen::Vector3d accel_static = -g;  // specific force when static
  const Eigen::Vector3d omega(0.0, 0.0, 0.0);
  const Eigen::Vector3d truth_pos(0.0, 0.0, 0.0);

  const Eigen::Vector3d p_init(0.5, -0.4, 0.3);
  auto X0 = TwoFrameGroup::FromState(
      Rot3::Identity(), Eigen::Vector3d(0.1, -0.05, 0.0), p_init,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  Cov15 P0 = Cov15::Identity() * 1e-4;             // confident attitude + bias
  P0.block<3, 3>(3, 3) = Cov3::Identity() * 0.25;  // velocity uncertainty
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 1.0;   // position uncertainty
  TFGInEKF ekf(X0, P0);

  Cov15 Qc = Cov15::Identity() * 1e-6;
  Cov3 R_pos = Cov3::Identity() * (0.05 * 0.05);
  const double dt = 0.01;  // 100 Hz IMU

  const double pos_trace_before = ekf.covariance().block<3, 3>(6, 6).trace();

  for (int k = 0; k < 500; ++k) {
    ekf.propagate(omega, accel_static, testGravity(), Qc, dt);
    if (k % 10 == 0)  // 10 Hz GNSS
      // Pin to C0: this test exercises the C0 coupling path. C* deliberately
      // zeroes the position->attitude coupling for a level vehicle, so it does
      // not drive horizontal velocity through the gravity feedback loop here.
      ekf.updatePosition(truth_pos, R_pos, /*use_cstar=*/false);
  }

  // Strong error reduction (> 60%) and clean velocity convergence.
  const double err_init = (p_init - truth_pos).norm();
  const double err_final = (ekf.position() - truth_pos).norm();
  EXPECT(err_final < 0.4 * err_init);
  EXPECT(veq(ekf.velocity(), Eigen::Vector3d::Zero(), 0.05));

  // Covariance stays SPD and the position block shrinks well below the prior.
  EXPECT(minEig(ekf.covariance()) > 0.0);
  const double pos_trace_after = ekf.covariance().block<3, 3>(6, 6).trace();
  EXPECT(pos_trace_after < 0.1 * pos_trace_before);
}

// ---------------------------------------------------------------------------
// C*: the midpoint-symmetrised output matrix (1/2 y_hat^) is the cubic-error
// linearisation. In this static-level geometry the position->attitude coupling
// is parallel to the innovation, so C* and C0 converge to the same fixed point
// (their difference is a subtle, regime-dependent transient effect, not a
// clean inequality). What must hold is that the default C* path yields a
// convergent, consistent filter even from a large initial error.
// ---------------------------------------------------------------------------

TEST(TFGInEKF, CstarConvergesFromLargeError) {
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  const Eigen::Vector3d accel_static = -g;
  const Eigen::Vector3d omega = Eigen::Vector3d::Zero();
  const Eigen::Vector3d truth_pos = Eigen::Vector3d::Zero();

  auto X0 = TwoFrameGroup::FromState(Rot3::Identity(), Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d(1.0, -0.8, 0.0),
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 0.05;
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 2.0;
  TFGInEKF ekf(X0, P0);

  Cov15 Qc = Cov15::Identity() * 1e-6;
  Cov3 R_pos = Cov3::Identity() * (0.05 * 0.05);
  const double dt = 0.01;
  for (int k = 0; k < 500; ++k) {
    ekf.propagate(omega, accel_static, testGravity(), Qc, dt);
    if (k % 10 == 0) ekf.updatePosition(truth_pos, R_pos, /*use_cstar=*/true);
  }
  EXPECT((ekf.position() - truth_pos).norm() < 0.01);
  EXPECT(minEig(ekf.covariance()) > 0.0);
}

// ---------------------------------------------------------------------------
// Predict builds nav<->bias cross-covariance: the predict step must couple
// nav and bias blocks, and a position update must therefore move the bias
// estimate. Before the fix the adjoint-only propagation left these blocks
// identically zero and biases were unobservable.
// ---------------------------------------------------------------------------

TEST(TFGInEKF, PredictBuildsNavBiasCrossCovariance) {
  auto X0 = TwoFrameGroup::FromState(
      Rot3::Rz(0.2), Eigen::Vector3d(0.3, -0.1, 0.0),
      Eigen::Vector3d(1.0, 0.5, -0.2), Eigen::Vector3d(0.01, -0.02, 0.0),
      Eigen::Vector3d(0.0, 0.03, -0.01));
  TFGInEKF ekf(X0, Cov15::Identity() * 0.1);  // diagonal: zero cross-cov

  ekf.propagate(Eigen::Vector3d(0.05, -0.02, 0.1),
                Eigen::Vector3d(0.0, 0.0, 9.81), testGravity(),
                Cov15::Identity() * 1e-6, 0.01);

  // Position(6..8) vs gyro-bias(9..11) and velocity(3..5) vs accel-bias(12..14)
  // cross-blocks are now populated by the lift Jacobian coupling.
  const Cov15 P = ekf.covariance();
  const double pos_bg = P.block<3, 3>(6, 9).norm();
  const double vel_ba = P.block<3, 3>(3, 12).norm();
  EXPECT(pos_bg > 1e-9);
  EXPECT(vel_ba > 1e-9);
}

TEST(TFGInEKF, PositionUpdateMovesBiasEstimate) {
  auto X0 = TwoFrameGroup::FromState(
      Rot3::Rz(0.2), Eigen::Vector3d(0.3, -0.1, 0.0),
      Eigen::Vector3d(1.0, 0.5, -0.2), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 0.1;
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 2.0;  // loose position
  TFGInEKF ekf(X0, P0);

  // One propagate to couple bias into the nav covariance, then a position fix.
  ekf.propagate(Eigen::Vector3d(0.05, -0.02, 0.1),
                Eigen::Vector3d(0.0, 0.0, 9.81), testGravity(),
                Cov15::Identity() * 1e-5, 0.01);
  const Eigen::Vector3d bg_before = ekf.biasGyro();
  const Eigen::Vector3d ba_before = ekf.biasAccel();

  ekf.updatePosition(Eigen::Vector3d(0.0, 0.0, 0.0), Cov3::Identity() * 1e-3);

  // Gain rows for the bias are no longer identically zero, so the estimate
  // moves. One 10 ms step from a diagonal prior only builds an O(dt^2)
  // nav-bias correlation, so the move is small (~6e-7) but non-zero.
  EXPECT((ekf.biasGyro() - bg_before).norm() +
             (ekf.biasAccel() - ba_before).norm() >
         1e-9);
}

// ---------------------------------------------------------------------------
// C* translation invariance: the equivariant residual / C* update must be
// invariant to the choice of world origin. Running an identical static
// scenario shifted by a large offset must give the same position-error
// trajectory. (The old 1/2 (y_hat+p_hat)^ coupling carried the absolute
// position p_hat and broke this.)
// ---------------------------------------------------------------------------

static double runStaticErrShifted(const Eigen::Vector3d& offset,
                                  bool use_cstar) {
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  const Eigen::Vector3d accel_static = -g;
  const Eigen::Vector3d omega = Eigen::Vector3d::Zero();
  const Eigen::Vector3d truth_pos = offset;

  auto X0 = TwoFrameGroup::FromState(
      Rot3::Identity(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, -0.8, 0.0) + offset,  // wrong position, shifted
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Identity() * 0.05;
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 2.0;
  TFGInEKF ekf(X0, P0);

  Cov15 Qc = Cov15::Identity() * 1e-6;
  Cov3 R_pos = Cov3::Identity() * (0.05 * 0.05);
  const double dt = 0.01;
  for (int k = 0; k < 500; ++k) {
    ekf.propagate(omega, accel_static, testGravity(), Qc, dt);
    if (k % 10 == 0) ekf.updatePosition(truth_pos, R_pos, use_cstar);
  }
  return (ekf.position() - truth_pos).norm();
}

TEST(TFGInEKF, CstarTranslationInvariance) {
  const double err0 = runStaticErrShifted(Eigen::Vector3d::Zero(), true);
  const double errS =
      runStaticErrShifted(Eigen::Vector3d(500.0, -300.0, 100.0), true);
  EXPECT_DOUBLES_EQUAL(err0, errS, 1e-6);
}

// ---------------------------------------------------------------------------
// Input noise couples into bias rows: the lifted process noise must couple
// gyro white noise into the bias rows (through b_w^/b_a^ in the input matrix
// B).
// ---------------------------------------------------------------------------

TEST(TFGInEKF, InputNoiseCouplesIntoBiasRows) {
  auto X = TwoFrameGroup::FromState(
      Rot3::Rz(0.3), Eigen::Vector3d(0.2, 0.0, -0.1),
      Eigen::Vector3d(0.5, -0.5, 0.1), Eigen::Vector3d(0.02, -0.01, 0.0),
      Eigen::Vector3d(0.0, 0.03, -0.02));
  tfg::ImuNoise noise;
  noise.gyro = Eigen::Vector3d::Constant(1e-2);  // gyro noise only
  const Cov15 Qd = tfg::inputNoiseCov(X, noise, 0.01);

  const double gw = Qd.block<3, 3>(9, 9).norm();     // gyro-bias rows driven
  const double ga = Qd.block<3, 3>(12, 12).norm();   // accel-bias rows driven
  const double th_gw = Qd.block<3, 3>(0, 9).norm();  // theta<->gyro-bias corr
  EXPECT(gw > 1e-9);
  EXPECT(ga > 1e-9);
  EXPECT(th_gw > 1e-9);
  EXPECT(assert_equal((Matrix)Qd, (Matrix)Qd.transpose(), 1e-15));  // symmetric
}

// ---------------------------------------------------------------------------
// 30 s regression: biased IMU, with and without position aiding.
//
// Ground truth is a coordinated turn (gtsam::ConstantTwistScenario: constant
// yaw rate + forward body velocity => circle). The turning motion excites the
// centripetal specific force, which makes yaw and the IMU biases observable
// from position fixes. IMU samples are synthesized from the closed-form
// trajectory and corrupted by CONSTANT biases only (no random noise), so the
// run is fully deterministic and thresholds are stable regression bounds.
// ---------------------------------------------------------------------------

struct RegressionResult {
  Eigen::Vector3d pos_err;         // final position error (m)
  Eigen::Vector3d vel_err;         // final velocity error (m/s)
  Eigen::Vector3d att_err;         // final attitude error, Log(gt^-1 est) (rad)
  Eigen::Vector3d bg_err, ba_err;  // final bias estimate errors
  Eigen::Vector3d bg_est, ba_est;  // final bias estimates
  double pos_cov_trace = 0.0;      // trace of final position covariance block
  double bg_cov_trace = 0.0;       // trace of final gyro-bias covariance block
  double ba_cov_trace = 0.0;       // trace of final accel-bias covariance block
  double min_eig = 0.0;            // min eigenvalue of final covariance
};

static RegressionResult runBiasedImu30s(bool with_position_updates) {
  // Coordinated turn: 0.3 rad/s yaw, 2 m/s forward => circle of radius ~6.7 m.
  const gtsam::ConstantTwistScenario scenario(Eigen::Vector3d(0, 0, 0.3),
                                              Eigen::Vector3d(2.0, 0, 0));
  // Constant true biases, unknown to the filter (it starts at zero).
  const Eigen::Vector3d true_bg(0.01, -0.005, 0.02);  // rad/s
  const Eigen::Vector3d true_ba(0.05, -0.03, 0.02);   // m/s^2

  const double dt = 0.01, duration = 30.0;  // 100 Hz IMU, 30 s
  const int num_steps = static_cast<int>(duration / dt);
  const int gnss_decim = 10;  // 10 Hz position fixes
  const Eigen::Vector3d g = testGravity();

  // Init at the true initial state with zero bias estimate.
  const gtsam::NavState gt0 = scenario.navState(0.0);
  auto X0 = TwoFrameGroup::FromState(gt0.attitude(), gt0.velocity(),
                                     gt0.position(), Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero());
  Cov15 P0 = Cov15::Zero();
  P0.block<3, 3>(0, 0) = Cov3::Identity() * 1e-4;    // attitude (confident)
  P0.block<3, 3>(3, 3) = Cov3::Identity() * 1e-2;    // velocity
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 1e-2;    // position
  P0.block<3, 3>(9, 9) = Cov3::Identity() * 1e-3;    // gyro bias
  P0.block<3, 3>(12, 12) = Cov3::Identity() * 1e-2;  // accel bias
  TFGInEKF ekf(X0, P0);

  tfg::ImuNoise noise;
  noise.gyro = Eigen::Vector3d::Constant(1e-6);   // (0.001 rad/s/sqrt(Hz))^2
  noise.accel = Eigen::Vector3d::Constant(1e-4);  // (0.01 m/s^2/sqrt(Hz))^2
  noise.gyro_rw = Eigen::Vector3d::Constant(1e-9);
  noise.accel_rw = Eigen::Vector3d::Constant(1e-8);
  const Cov3 R_pos = Cov3::Identity() * (0.05 * 0.05);

  double t = 0.0;
  for (int k = 0; k < num_steps; ++k) {
    // Ideal IMU at time t (specific force f = R^T (a_n - g)) plus the biases.
    const gtsam::NavState gt = scenario.navState(t);
    const Eigen::Vector3d omega_meas = scenario.omega_b(t) + true_bg;
    const Eigen::Vector3d accel_meas =
        gt.attitude().unrotate(scenario.acceleration_n(t) - g) + true_ba;
    ekf.propagate(omega_meas, accel_meas, testGravity(), noise, dt);
    t += dt;
    if (with_position_updates && (k + 1) % gnss_decim == 0)
      ekf.updatePosition(scenario.navState(t).position(), R_pos);
  }

  const gtsam::NavState gt_end = scenario.navState(duration);
  RegressionResult r;
  r.pos_err = Eigen::Vector3d(gt_end.position()) - ekf.position();
  r.vel_err = Eigen::Vector3d(gt_end.velocity()) - ekf.velocity();
  r.att_err = gtsam::Rot3::Logmap(gt_end.attitude().between(ekf.attitude()));
  r.bg_est = ekf.biasGyro();
  r.ba_est = ekf.biasAccel();
  r.bg_err = r.bg_est - true_bg;
  r.ba_err = r.ba_est - true_ba;
  const Cov15 P = ekf.covariance();
  r.pos_cov_trace = P.block<3, 3>(6, 6).trace();
  r.bg_cov_trace = P.block<3, 3>(9, 9).trace();
  r.ba_cov_trace = P.block<3, 3>(12, 12).trace();
  r.min_eig = minEig(P);
  return r;
}

// Pretty-print the regression metrics (regression tests only).
static void printRegression(const std::string& tag, const RegressionResult& r) {
  printf(
      "[%s] pos=%.4e m  vel=%.4e m/s  att=%.4e rad  "
      "bg_err=%.4e  ba_err=%.4e  ba_err.z=%.4e  sigma_ba=%.4e  "
      "pos_cov=%.4e  bg_cov=%.4e  min_eig=%.4e\n",
      tag.c_str(), r.pos_err.norm(), r.vel_err.norm(), r.att_err.norm(),
      r.bg_err.norm(), r.ba_err.norm(), std::abs(r.ba_err.z()),
      std::sqrt(r.ba_cov_trace / 3.0), r.pos_cov_trace, r.bg_cov_trace,
      r.min_eig);
}

TEST(TFGInEKF, Regression30sBiasedImuWithPositionUpdates) {
  const RegressionResult r = runBiasedImu30s(/*with_position_updates=*/true);
  printRegression("aided", r);

  // Navigation errors stay small under aiding despite the uncompensated-at-
  // start biases (5 cm GNSS-grade bound on position). Measured (Release,
  // x86-64): pos 3.1e-4 m, vel 6.3e-4 m/s, att 4.3e-3 rad.
  EXPECT(r.pos_err.norm() < 0.05);
  EXPECT(r.vel_err.norm() < 0.05);
  EXPECT(r.att_err.norm() < 0.02);  // ~1.1 deg

  // Gyro bias is recovered almost fully (truth |bg| = 0.023; measured error
  // 8.8e-4). The accel bias splits by observability: the z component (paired
  // with gravity) recovers to ~1e-3, while the horizontal components remain
  // entangled with the few-mrad tilt (g * tilt ~ 0.02 m/s^2) -- the filter's
  // own sigma_ba ~ 0.06 covers this, i.e. it is consistent, not divergent.
  // Truth |ba| = 0.062; measured total error 0.030.
  EXPECT(r.bg_err.norm() < 0.005);
  EXPECT(r.ba_err.norm() < 0.045);         // > 25% recovery overall
  EXPECT(std::abs(r.ba_err.z()) < 0.005);  // observable axis: tight
  const double sigma_ba = std::sqrt(r.ba_cov_trace / 3.0);
  EXPECT(r.ba_err.norm() < 3.0 * sigma_ba);  // filter consistency

  // Covariance stays SPD; the gyro-bias block converges well below its prior.
  EXPECT(r.min_eig > 0.0);
  EXPECT(r.bg_cov_trace < 0.1 * 3e-3);
}

TEST(TFGInEKF, Regression30sAidedVsImuOnly) {
  const RegressionResult aided =
      runBiasedImu30s(/*with_position_updates=*/true);
  const RegressionResult imu_only =
      runBiasedImu30s(/*with_position_updates=*/false);
  printRegression("aided", aided);
  printRegression("imu_only", imu_only);

  // Dead reckoning with a biased IMU drifts far (measured: 152 m after 30 s);
  // aiding contains the error to sub-cm.
  EXPECT(imu_only.pos_err.norm() > 10.0);
  EXPECT(aided.pos_err.norm() < 0.01 * imu_only.pos_err.norm());

  // Without aiding the biases are unobservable: the estimate never moves from
  // its zero initialisation (propagate keeps the physical bias constant).
  EXPECT(imu_only.bg_est.norm() < 1e-6);
  EXPECT(imu_only.ba_est.norm() < 1e-6);

  // The filter knows it: position covariance grows instead of shrinking.
  EXPECT(imu_only.pos_cov_trace > 10.0 * aided.pos_cov_trace);
  EXPECT(imu_only.min_eig > 0.0);
}

/* ************************************************************************* */
namespace depth_update {

using Cov1 = TFGInEKF::Covariance1;

// 1x1 depth noise matrix from a stddev in metres.
Cov1 depthNoise(double sigma) {
  Cov1 R;
  R(0, 0) = sigma * sigma;
  return R;
}

// Tilted state: R_meas = R^T R_pseudo R is only diagonal when R is, so a roll
// keeps that transport honest.
TwoFrameGroup tiltedState() {
  return TwoFrameGroup::FromState(
      Rot3::Rx(M_PI / 6), Eigen::Vector3d(0.5, -0.2, 0.1),
      Eigen::Vector3d(1.0, 2.0, -3.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());
}

// A single depth update pulls the vertical estimate most of the way to the
// measurement when the depth noise is much tighter than the prior.
TEST(TFGInEKF, DepthUpdateCorrectsVerticalPosition) {
  TFGInEKF ekf(TwoFrameGroup::Identity(), Cov15::Identity() * 0.01);

  const double z_depth = -0.8;
  ekf.updateDepth(z_depth, depthNoise(0.01));

  EXPECT(std::abs(ekf.position().z() - z_depth) < 0.1);
}

// The inflated horizontal variance keeps a depth update off the x and y axes
// when the prior is uncorrelated.
TEST(TFGInEKF, DepthUpdateLeavesHorizontalNearlyUnchanged) {
  TFGInEKF ekf(TwoFrameGroup::Identity(), Cov15::Identity() * 0.01);

  const Eigen::Vector3d before = ekf.position();
  ekf.updateDepth(-0.8, depthNoise(0.01));
  const Eigen::Vector3d delta = ekf.position() - before;

  EXPECT(std::abs(delta.z()) > 0.5);
  EXPECT(std::abs(delta.x()) < 0.02);
  EXPECT(std::abs(delta.y()) < 0.02);
}

// Only the vertical position covariance is informed; the horizontal variances
// gain no information. The covariance reset may still grow them: in the group
// chart the position error couples to attitude through the correction eta_p,
// so a large vertical correction transports attitude variance onto x and y.
TEST(TFGInEKF, DepthUpdateShrinksVerticalCovariance) {
  TFGInEKF ekf(TwoFrameGroup::Identity(), Cov15::Identity() * 0.01);

  const Cov15 before = ekf.covariance();
  ekf.updateDepth(-0.8, depthNoise(0.01));
  const Cov15 after = ekf.covariance();

  const double drop_z = before(8, 8) - after(8, 8);
  EXPECT(drop_z > 0.0);
  EXPECT(after(6, 6) > before(6, 6) - 0.1 * drop_z);
  EXPECT(after(7, 7) > before(7, 7) - 0.1 * drop_z);
  EXPECT(minEig(after) > 0.0);
}

// updateDepth is exactly the pseudo-position update it documents. Checked at a
// tilted state, where the R^T(.)R noise transport is not a no-op.
TEST(TFGInEKF, DepthUpdateMatchesEquivalentPositionUpdate) {
  TFGInEKF via_depth(tiltedState(), Cov15::Identity() * 0.01);
  TFGInEKF via_position(tiltedState(), Cov15::Identity() * 0.01);

  const double z_depth = -2.0;
  const double sigma_z = 0.02;
  via_depth.updateDepth(z_depth, depthNoise(sigma_z));

  Cov3 R_pseudo = Cov3::Zero();
  R_pseudo(0, 0) = TFGInEKF::kDefaultHorizontalVariance;
  R_pseudo(1, 1) = TFGInEKF::kDefaultHorizontalVariance;
  R_pseudo(2, 2) = sigma_z * sigma_z;
  Eigen::Vector3d pseudo = via_position.position();
  pseudo.z() = z_depth;
  via_position.updatePosition(pseudo, R_pseudo);

  EXPECT(traits<TwoFrameGroup>::Equals(via_position.state(), via_depth.state(),
                                       1e-12));
  EXPECT(assert_equal((Matrix)via_position.covariance(),
                      (Matrix)via_depth.covariance(), 1e-12));
}

// At a tilted state with a metre-sized vertical offset the update still moves
// the estimate toward the measured depth; one update against a decimetre prior
// cannot close the gap, so only a strict decrease is asserted.
TEST(TFGInEKF, DepthUpdateAtTiltedStateReducesVerticalError) {
  TFGInEKF ekf(tiltedState(), Cov15::Identity() * 0.01);

  const double z_depth = ekf.position().z() + 1.0;
  const double err_before = std::abs(ekf.position().z() - z_depth);
  ekf.updateDepth(z_depth, depthNoise(0.01));
  const double err_after = std::abs(ekf.position().z() - z_depth);

  EXPECT(err_after < 0.95 * err_before);
}

// The horizontal-variance argument is live: shrinking it changes the correction
// a correlated prior produces, so the default is not silently ignored.
TEST(TFGInEKF, DepthUpdateHorizontalVarianceIsRespected) {
  // Correlate p_x with p_z; the block [[0.01, 0.008], [0.008, 0.01]] is SPD.
  Cov15 P0 = Cov15::Identity() * 0.01;
  P0(6, 8) = P0(8, 6) = 0.008;

  TFGInEKF inflated(TwoFrameGroup::Identity(), P0);
  TFGInEKF pinned(TwoFrameGroup::Identity(), P0);

  const double z_depth = -0.8;
  inflated.updateDepth(z_depth, depthNoise(0.01));
  pinned.updateDepth(z_depth, depthNoise(0.01), /*horizontal_variance=*/1e-6);

  EXPECT((inflated.position() - pinned.position()).norm() > 1e-3);
}

// The direct depth Jacobian is the derivative of p_z along the right-retract
// chart, e_3^T R on the position block, checked at a tilted state.
TEST(TFGInEKF, DepthOutputJacobianMatchesNumericalDerivative) {
  using Vector15 = TwoFrameGroup::TangentVector;
  const TwoFrameGroup X = tiltedState();
  const std::function<double(const Vector15&)> h = [&](const Vector15& eps) {
    return DepthOutput::predict(traits<TwoFrameGroup>::Retract(X, eps));
  };
  const Matrix expected =
      numericalDerivative11<double, Vector15>(h, Vector15::Zero());
  EXPECT(assert_equal(expected, (Matrix)DepthOutput::jacobian(X), 1e-8));
}

// With an uncorrelated prior the direct update moves z alone and adds no
// horizontal information. The reset may still grow the x and y variances, as
// for the pseudo-measurement above.
TEST(TFGInEKF, DirectDepthUpdateAddsNoHorizontalInformation) {
  TFGInEKF ekf(TwoFrameGroup::Identity(), Cov15::Identity() * 0.01);

  const Eigen::Vector3d before = ekf.position();
  const Cov15 P_before = ekf.covariance();
  const double z_depth = -0.8;
  ekf.updateDepthDirect(z_depth, depthNoise(0.01));
  const Cov15 P_after = ekf.covariance();

  EXPECT(std::abs(ekf.position().z() - z_depth) < 0.1);
  EXPECT_DOUBLES_EQUAL(before.x(), ekf.position().x(), 1e-12);
  EXPECT_DOUBLES_EQUAL(before.y(), ekf.position().y(), 1e-12);
  EXPECT(P_after(8, 8) < P_before(8, 8));
  EXPECT(P_after(6, 6) >= P_before(6, 6) - 1e-12);
  EXPECT(P_after(7, 7) >= P_before(7, 7) - 1e-12);
  EXPECT(minEig(P_after) > 0.0);
}

// Stripped of horizontal weight, the pseudo-position update reduces to the
// scalar model: its world-z row is e_3^T R C, whose attitude part vanishes
// because a^T [a]_x = 0 for a = R^T e_3. The two updates therefore coincide
// as the horizontal variance grows, even at a tilted state. The variance stays
// at 1e6 because the body-frame noise R^T diag(h, h, sigma_z^2) R mixes h into
// the z entry, so a much larger h would round sigma_z^2 away.
TEST(TFGInEKF,
     DirectDepthUpdateMatchesThePseudoPositionUpdateInTheInflatedLimit) {
  TFGInEKF via_pseudo(tiltedState(), Cov15::Identity() * 0.01);
  TFGInEKF via_direct(tiltedState(), Cov15::Identity() * 0.01);

  const double z_depth = -2.0;
  const double sigma_z = 0.02;
  via_pseudo.updateDepth(z_depth, depthNoise(sigma_z),
                         /*horizontal_variance=*/1e6);
  via_direct.updateDepthDirect(z_depth, depthNoise(sigma_z));

  EXPECT(traits<TwoFrameGroup>::Equals(via_pseudo.state(), via_direct.state(),
                                       1e-7));
  EXPECT(assert_equal((Matrix)via_pseudo.covariance(),
                      (Matrix)via_direct.covariance(), 1e-7));
}

}  // namespace depth_update
/* ************************************************************************* */

/* ************************************************************************* */
namespace covariance_reset {

using Vector15 = TwoFrameGroup::TangentVector;

TwoFrameGroup rotatedState() {
  return TwoFrameGroup::FromState(
      Rot3::RzRyRx(0.3, -0.2, 0.5), Eigen::Vector3d(0.5, -0.2, 0.1),
      Eigen::Vector3d(1.0, 2.0, -3.0), Eigen::Vector3d(0.01, -0.02, 0.03),
      Eigen::Vector3d(0.1, 0.0, -0.1));
}

// Prior with attitude-position correlation, so a position fix also corrects
// attitude and exercises the reset.
Cov15 correlatedPrior() {
  Cov15 P0 = Cov15::Identity() * 0.01;
  P0.block<3, 3>(6, 6) = Cov3::Identity() * 1.0;
  P0.block<3, 3>(0, 6) = Cov3::Identity() * 0.05;
  P0.block<3, 3>(6, 0) = Cov3::Identity() * 0.05;
  return P0;
}

// resetJacobian(eta) is the derivative of e -> Local(X * Expmap(eta),
// X * Expmap(e)) at the posterior mean e = eta.
TEST(TFGInEKF, ResetJacobianMatchesNumericalDerivative) {
  const TwoFrameGroup X = rotatedState();
  Vector15 eta;
  eta << 0.2, -0.1, 0.3, 0.1, 0.2, -0.1, 0.5, 0.4, -0.3, 0.01, -0.02, 0.03,
      0.05, -0.04, 0.02;

  const TwoFrameGroup X_plus = traits<TwoFrameGroup>::Retract(X, eta);
  Cov15 numerical;
  const double h = 1e-6;
  for (int j = 0; j < 15; ++j) {
    Vector15 step = Vector15::Zero();
    step(j) = h;
    const Vector15 plus = traits<TwoFrameGroup>::Local(
        X_plus, traits<TwoFrameGroup>::Retract(X, eta + step));
    const Vector15 minus = traits<TwoFrameGroup>::Local(
        X_plus, traits<TwoFrameGroup>::Retract(X, eta - step));
    numerical.col(j) = (plus - minus) / (2 * h);
  }

  EXPECT(assert_equal((Matrix)numerical, (Matrix)TFGInEKF::resetJacobian(eta),
                      1e-8));
}

/// One measurement update from rotatedState() and correlatedPrior(), run by the
/// filter and, with the reset skipped, by the base InvariantEKF.
struct ResetCheck {
  bool same_state;      // both reach the same corrected estimate
  Vector15 eta;         // the correction the update applied
  Cov15 untransported;  // Joseph-updated covariance, before the reset
  Cov15 expected;       // its reset transport B P B^T
  Cov15 filtered;       // the covariance the filter ships
};

ResetCheck resetCheck(
    const std::function<void(TFGInEKF&)>& update,
    const std::function<void(gtsam::InvariantEKF<TwoFrameGroup>&)>&
        baseUpdate) {
  const TwoFrameGroup X0 = rotatedState();
  TFGInEKF ekf(X0, correlatedPrior());
  update(ekf);
  gtsam::InvariantEKF<TwoFrameGroup> base(X0, correlatedPrior());
  baseUpdate(base);

  const Vector15 eta = traits<TwoFrameGroup>::Local(X0, base.state());
  const Cov15 B = TFGInEKF::resetJacobian(eta);
  return {traits<TwoFrameGroup>::Equals(base.state(), ekf.state(), 1e-12), eta,
          base.covariance(), B * base.covariance() * B.transpose(),
          ekf.covariance()};
}

/// updatePosition without the reset: body-frame residual and rotated noise.
void basePositionUpdate(gtsam::InvariantEKF<TwoFrameGroup>& base,
                        const Vector3& pi, const Cov3& R_pos) {
  const TwoFrameGroup X = base.state();
  const Matrix3 R = X.R.matrix();
  base.updateWithVector(PositionOutput::predict(X, pi),
                        PositionOutput::jacobian(X, pi), Vector3::Zero(),
                        R.transpose() * R_pos * R, /*performReset=*/false);
}

// A position update transports the Joseph-updated covariance by the reset
// Jacobian of the applied correction, unlike the untransported base update.
TEST(TFGInEKF, PositionUpdateAppliesCovarianceReset) {
  const Eigen::Vector3d fix(1.6, 1.5, -2.6);
  const Cov3 R_pos = Cov3::Identity() * 0.01;
  const ResetCheck c =
      resetCheck([&](TFGInEKF& ekf) { ekf.updatePosition(fix, R_pos); },
                 [&](gtsam::InvariantEKF<TwoFrameGroup>& base) {
                   basePositionUpdate(base, fix, R_pos);
                 });

  EXPECT(c.same_state);
  EXPECT(c.eta.head<3>().norm() > 1e-3);  // the fix corrected attitude
  EXPECT(assert_equal((Matrix)c.expected, (Matrix)c.filtered, 1e-12));
  EXPECT((c.untransported - c.filtered).norm() > 1e-6);
}

// The DVL update applies the same reset.
TEST(TFGInEKF, DvlUpdateAppliesCovarianceReset) {
  const Eigen::Vector3d z_dvl(0.8, -0.5, 0.3);
  const Cov3 R_dvl = Cov3::Identity() * 0.01;
  const ResetCheck c = resetCheck(
      [&](TFGInEKF& ekf) { ekf.updateDvl(z_dvl, R_dvl); },
      [&](gtsam::InvariantEKF<TwoFrameGroup>& base) {
        base.updateWithVector(BodyVelocityOutput::predict(base.state()),
                              BodyVelocityOutput::jacobian(base.state()), z_dvl,
                              R_dvl, /*performReset=*/false);
      });

  EXPECT(c.same_state);
  EXPECT(c.eta.head<3>().norm() > 1e-3);
  EXPECT(assert_equal((Matrix)c.expected, (Matrix)c.filtered, 1e-12));
  EXPECT((c.untransported - c.filtered).norm() > 1e-6);
}

// The depth update, a pseudo-position update, applies the same reset.
TEST(TFGInEKF, DepthUpdateAppliesCovarianceReset) {
  const double z_depth = -2.6;
  TFGInEKF::Covariance1 R_depth;
  R_depth(0, 0) = 0.01;
  const ResetCheck c = resetCheck(
      [&](TFGInEKF& ekf) { ekf.updateDepth(z_depth, R_depth); },
      [&](gtsam::InvariantEKF<TwoFrameGroup>& base) {
        Eigen::Vector3d pseudo_position = base.state().p;
        pseudo_position.z() = z_depth;
        const Cov3 R_pseudo =
            Eigen::Vector3d(TFGInEKF::kDefaultHorizontalVariance,
                            TFGInEKF::kDefaultHorizontalVariance, R_depth(0, 0))
                .asDiagonal();
        basePositionUpdate(base, pseudo_position, R_pseudo);
      });

  EXPECT(c.same_state);
  EXPECT(c.eta.head<3>().norm() > 1e-3);
  EXPECT(assert_equal((Matrix)c.expected, (Matrix)c.filtered, 1e-12));
  EXPECT((c.untransported - c.filtered).norm() > 1e-6);
}

}  // namespace covariance_reset
/* ************************************************************************* */

/* ************************************************************************* */
namespace transition_jacobian {

using Vector15 = TwoFrameGroup::TangentVector;

// The discrete step the filter propagates the mean with.
TwoFrameGroup step(const TwoFrameGroup& X, const ImuInput& u, double dt) {
  return X * increment(X, u, dt);
}

// transitionJacobian is the exact derivative of the implemented discrete step,
// not a first-order-in-dt approximation: checked at a large dt with fast
// rotation and non-zero biases, where the O(dt^2) term is ~1e-2.
TEST(TFGInEKF, TransitionJacobianMatchesFiniteDifference) {
  const TwoFrameGroup X = TwoFrameGroup::FromState(
      Rot3::RzRyRx(0.3, -0.2, 0.5), Eigen::Vector3d(0.5, -0.2, 0.1),
      Eigen::Vector3d(1.0, 2.0, -3.0), Eigen::Vector3d(0.05, -0.04, 0.03),
      Eigen::Vector3d(0.1, 0.2, -0.1));
  ImuInput u;
  u.omega = Eigen::Vector3d(0.6, -0.4, 0.8);
  u.accel = Eigen::Vector3d(0.3, -0.2, 9.9);
  u.g_vec = testGravity();
  const double dt = 0.2;

  const TwoFrameGroup X_next = step(X, u, dt);
  Cov15 numerical;
  const double h = 1e-6;
  for (int j = 0; j < 15; ++j) {
    Vector15 delta = Vector15::Zero();
    delta(j) = h;
    const Vector15 plus = traits<TwoFrameGroup>::Local(
        X_next, step(traits<TwoFrameGroup>::Retract(X, delta), u, dt));
    const Vector15 minus = traits<TwoFrameGroup>::Local(
        X_next, step(traits<TwoFrameGroup>::Retract(X, -delta), u, dt));
    numerical.col(j) = (plus - minus) / (2 * h);
  }

  EXPECT(assert_equal((Matrix)numerical, (Matrix)transitionJacobian(X, u, dt),
                      1e-7));
}

// propagate() uses that Jacobian: with no process noise, P1 = F P0 F^T.
TEST(TFGInEKF, PropagateUsesTransitionJacobian) {
  const TwoFrameGroup X0 = TwoFrameGroup::FromState(
      Rot3::Rz(0.4), Eigen::Vector3d(0.2, 0.1, 0.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.02, 0.01, -0.03), Eigen::Vector3d(0.05, 0.0, 0.1));
  Cov15 P0 = Cov15::Identity() * 0.01;
  P0.block<3, 3>(0, 9) = P0.block<3, 3>(9, 0) = Cov3::Identity() * 0.002;
  ImuInput u;
  u.omega = Eigen::Vector3d(0.3, -0.2, 0.5);
  u.accel = Eigen::Vector3d(0.1, 0.0, 9.8);
  u.g_vec = testGravity();
  const double dt = 0.1;

  TFGInEKF ekf(X0, P0);
  ekf.propagate(u.omega, u.accel, u.g_vec, Cov15::Zero(), dt);

  const Cov15 F = transitionJacobian(X0, u, dt);
  EXPECT(assert_equal((Matrix)(F * P0 * F.transpose()),
                      (Matrix)ekf.covariance(), 1e-12));
}

}  // namespace transition_jacobian
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
