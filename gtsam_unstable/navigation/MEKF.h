/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MEKF.h
 * @brief Multiplicative EKF (MEKF) baseline for biased inertial navigation.
 * @author Arihant Lunawat
 *
 * State-dependent F (finite-diff); naive position/DVL updates vs geometric
 * filters. The process model (Dynamics) and measurement outputs live here;
 * the state manifold is in MEKFGeometry.h.
 */

#pragma once
#include <gtsam/navigation/ManifoldEKF.h>
#include <gtsam_unstable/navigation/MEKFGeometry.h>

#include <Eigen/Dense>

namespace gtsam {
namespace mekf {

struct ImuInput {
  Vector3 omega = Vector3::Zero();
  Vector3 accel = Vector3::Zero();

  /// Gravity vector in the navigation frame. The filter is frame-agnostic: the
  /// caller's choice of sign is what fixes which way +z points (Z-down/NED
  /// gives +g on z, Z-up/ENU gives -g). Defaults to zero, i.e. no gravity.
  Vector3 g_vec = Vector3::Zero();
};

GTSAM_UNSTABLE_EXPORT MEKFState propagateMean(const MEKFState& X,
                                              const ImuInput& u, double dt);

/// Finite-difference transition Jacobian F (state-dependent, recomputed each
/// step).
GTSAM_UNSTABLE_EXPORT MEKFState::Jacobian transitionJacobian(const MEKFState& X,
                                                             const ImuInput& u,
                                                             double dt);

/// Body-frame velocity output: h(xi) = R^T v.
/// Jacobian w.r.t. tangent under R_true = R_hat * Exp(d_theta):
///   d(R^T v)/d(d_theta) = [R^T v]^x,  d(R^T v)/d(d_v) = R^T
///   H = [ [R^T v]^x | R^T | 0 | 0 | 0 ]
struct GTSAM_UNSTABLE_EXPORT BodyVelocityOutput {
  using Jacobian = Eigen::Matrix<double, 3, 15>;

  static Vector3 predict(const MEKFState& X);
  static Jacobian jacobian(const MEKFState& X);
};

/// Direct global position output: h(xi) = p.
struct GTSAM_UNSTABLE_EXPORT PositionOutput {
  using Jacobian = Eigen::Matrix<double, 3, 15>;

  static Vector3 predict(const MEKFState& X);
  static Jacobian jacobian();
};

/// Direct depth output: h(xi) = e_3^T p, the world-frame z position.
struct GTSAM_UNSTABLE_EXPORT DepthOutput {
  using Jacobian = Eigen::Matrix<double, 1, 15>;

  static double predict(const MEKFState& X);
  /// e_3^T on the additive position block, zero elsewhere.
  static Jacobian jacobian();
};

/// Continuous-time IMU noise PSDs, per-axis (diagonal, not isotropic). Same
/// convention as tfg::ImuNoise / gtsam::tgeqf::ImuNoise.
struct ImuNoise {
  Vector3 gyro = Vector3::Zero();
  Vector3 accel = Vector3::Zero();
  Vector3 gyro_rw = Vector3::Zero();
  Vector3 accel_rw = Vector3::Zero();
};

class GTSAM_UNSTABLE_EXPORT MEKF : public gtsam::ManifoldEKF<MEKFState> {
 public:
  using Base = gtsam::ManifoldEKF<MEKFState>;
  using Covariance = Eigen::Matrix<double, 15, 15>;
  using Covariance3 = Eigen::Matrix<double, 3, 3>;
  using Covariance1 = Eigen::Matrix<double, 1, 1>;

  /// Default variance (m^2) given to the pseudo-measured horizontal axes of a
  /// depth update, large enough that the update leaves x and y untouched.
  static constexpr double kDefaultHorizontalVariance = 1e6;

  /// @param X0 initial state.
  /// @param P0 initial covariance in the MEKF chart at X0, whose attitude
  ///           error is body-frame (see initialCovariance for a physical
  ///           prior).
  MEKF(const MEKFState& X0, const Covariance& P0);

  /**
   * Express a prior over navigation errors in the MEKF chart at X0. The
   * navigation error [dtheta, dv, dp, db_omega, db_accel] relates the true
   * state to the estimate X0 by R = Exp(dtheta) R0 (a world-frame rotation),
   * v = v0 + dv, p = p0 + dp and b = b0 + db, so Sigma_navigation is stated in
   * physical units on world axes. The MEKF chart differs only in its attitude
   * error, R = R0 Exp(delta), so the result is J Sigma J^T with
   * J = diag(R0^T, I, I, I, I).
   */
  static Covariance initialCovariance(const MEKFState& X0,
                                      const Covariance& Sigma_navigation);

  /**
   * Strapdown predict + finite-diff F; P <- F P F^T + Qc dt.
   *
   * Qc is the process-noise PSD already expressed in the filter's tangent
   * chart (block order R, v, p, bg, ba), so its velocity block is in the
   * navigation frame. Use the ImuNoise overload to supply raw IMU PSDs.
   *
   * @param g_vec  Nav-frame gravity vector; its sign fixes the frame
   *               convention (Z-down/NED: +g on z).
   */
  void propagate(const Vector3& omega, const Vector3& accel,
                 const Vector3& g_vec, const Covariance& Qc, double dt);

  /**
   * Same predict with Qc built from body-frame IMU PSDs.
   *
   * Gyro noise enters the right-multiplicative attitude chart unrotated, but
   * accelerometer noise enters the world-frame velocity as R n_a, so the
   * velocity block is R diag(accel) R^T, evaluated at the pre-step attitude.
   */
  void propagate(const Vector3& omega, const Vector3& accel,
                 const Vector3& g_vec, const ImuNoise& noise, double dt);

  /// GNSS position update (h = p, world-frame R_pos).
  void updatePosition(const Vector3& pi, const Covariance3& R_pos);

  /// DVL update (h = R^T v, state-dependent H, body-frame R_dvl).
  void updateDvl(const Vector3& z_dvl, const Covariance3& R_dvl);

  /**
   * Pressure-sensor depth update from the world-frame z position.
   *
   * The depth is stacked onto the estimated horizontal position to form the
   * pseudo-position p_tilde = [p_x, p_y, z_depth]^T, which is fed through the
   * position output with the horizontal variance inflated so only the vertical
   * direction is corrected. This is the depth
   * pseudo-measurement construction of E. R. Potokar, K. Norman, and
   * J. G. Mangelson, "Invariant Extended Kalman Filtering for Underwater
   * Navigation," IEEE Robotics and Automation Letters, vol. 6, no. 3, 2021,
   * doi:10.1109/LRA.2021.3085167.
   * The same route is used in all three filters so they share one API.
   *
   * @param z_depth              Measured world-frame z position.
   * @param R_depth              Variance of z_depth, as a 1x1 matrix.
   * @param horizontal_variance  Variance assigned to the pseudo-measured x and
   *                             y axes.
   */
  void updateDepth(double z_depth, const Covariance1& R_depth,
                   double horizontal_variance = kDefaultHorizontalVariance);

  /**
   * Pressure-sensor depth update through the scalar model h(xi) = e_3^T p
   * (see DepthOutput).
   *
   * Unlike updateDepth(), no horizontal pseudo-measurement is formed: the
   * Jacobian has no horizontal component, so x and y move only through their
   * prior correlation with z.
   *
   * @param z_depth  Measured world-frame z position.
   * @param R_depth  Variance of z_depth, as a 1x1 matrix.
   */
  void updateDepthDirect(double z_depth, const Covariance1& R_depth);

  /// Error state eps = Local(estimate, X_true) in the filter's local chart
  /// (right/body multiplicative attitude, additive vectors), i.e. the chart
  /// covariance() lives in. Block order R, v, p, bg, ba.
  TangentVector errorStateVector(const MEKFState& X_true) const;

  gtsam::Rot3 attitude() const;
  Vector3 velocity() const;
  Vector3 position() const;
  Vector3 biasGyro() const;
  Vector3 biasAccel() const;

  /**
   * Covariance reset Jacobian for a correction eta applied by Retract.
   *
   * The posterior error e ~ N(eta, P) satisfies X_true = Retract(X, e); after
   * the correction the error is e' = Local(Retract(X, eta), X_true), whose
   * derivative at e = eta is diag(J_r(eta_theta), I_12), J_r the SO(3) right
   * Jacobian. This is the posterior-mean reset, the same convention as
   * TGEqF::resetMatrix.
   */
  static Jacobian resetJacobian(const TangentVector& eta);

 private:
  /// Kalman update in the current chart followed by the covariance reset
  /// P <- B P B^T with B = resetJacobian(eta).
  void updateWithReset(const gtsam::Vector& prediction, const gtsam::Matrix& H,
                       const gtsam::Vector& z, const gtsam::Matrix& R);
};

}  // namespace mekf
}  // namespace gtsam
