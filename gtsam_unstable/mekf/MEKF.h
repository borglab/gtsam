#pragma once
#include <gtsam/navigation/ManifoldEKF.h>
#include <gtsam_unstable/mekf/MEKFGeometry.h>

#include <Eigen/Dense>

// MultiplicativeEKF: baseline MEKF for biased INS. See mekf/README.md.
// State-dependent F (finite-diff); naive position/DVL updates vs geometric
// filters. The process model (Dynamics) and measurement outputs live here;
// the state manifold is in MEKFGeometry.h.

namespace mekf {

struct ImuInput {
  Vector3 omega = Vector3::Zero();
  Vector3 accel = Vector3::Zero();

  /// Gravity vector in the navigation frame. The filter is frame-agnostic: the
  /// caller's choice of sign is what fixes which way +z points (Z-down/NED
  /// gives +g on z, Z-up/ENU gives -g). Defaults to zero, i.e. no gravity.
  Vector3 g_vec = Vector3::Zero();
};

GTSAM_UNSTABLE_EXPORT MekfState propagateMean(const MekfState& X,
                                              const ImuInput& u, double dt);

/// Finite-difference transition Jacobian F (state-dependent, recomputed each
/// step).
GTSAM_UNSTABLE_EXPORT MekfState::Jacobian transitionJacobian(const MekfState& X,
                                                             const ImuInput& u,
                                                             double dt);

/// Body-frame velocity output: h(xi) = R^T v.
/// Jacobian w.r.t. tangent under R_true = R_hat * Exp(d_theta):
///   d(R^T v)/d(d_theta) = [R^T v]^x,  d(R^T v)/d(d_v) = R^T
///   H = [ [R^T v]^x | R^T | 0 | 0 | 0 ]
struct GTSAM_UNSTABLE_EXPORT BodyVelocityOutput {
  using Jacobian = Eigen::Matrix<double, 3, 15>;

  static Vector3 predict(const MekfState& X);
  static Jacobian jacobian(const MekfState& X);
};

/// Direct global position output: h(xi) = p.
struct GTSAM_UNSTABLE_EXPORT PositionOutput {
  using Jacobian = Eigen::Matrix<double, 3, 15>;

  static Vector3 predict(const MekfState& X);
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

class GTSAM_UNSTABLE_EXPORT MultiplicativeEKF
    : public gtsam::ManifoldEKF<MekfState> {
 public:
  using Base = gtsam::ManifoldEKF<MekfState>;
  using Covariance = Eigen::Matrix<double, 15, 15>;
  using Covariance3 = Eigen::Matrix<double, 3, 3>;
  using Covariance1 = Eigen::Matrix<double, 1, 1>;

  /// Default variance (m^2) given to the pseudo-measured horizontal axes of a
  /// depth update, large enough that the update leaves x and y untouched.
  static constexpr double kDefaultHorizontalVariance = 1e3;

  MultiplicativeEKF(const MekfState& X0, const Covariance& P0);

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
  void update_position(const Vector3& pi, const Covariance3& R_pos);

  /// DVL update (h = R^T v, state-dependent H, body-frame R_dvl).
  void update_dvl(const Vector3& z_dvl, const Covariance3& R_dvl);

  /**
   * Pressure-sensor depth update from the world-frame z position.
   *
   * The depth is stacked onto the estimated horizontal position to form the
   * pseudo-position p_tilde = [p_x, p_y, z_depth]^T, which is fed through the
   * position output with the horizontal variance inflated so only the vertical
   * direction is corrected. The same pseudo-measurement route is used in all
   * three filters so they share one API; see Sec. V-B of
   * tg_eqf/docs/EqF_design_for_DVL_Depth_aided_INS.pdf.
   *
   * @param z_depth              Measured world-frame z position.
   * @param R_depth              Variance of z_depth, as a 1x1 matrix.
   * @param horizontal_variance  Variance assigned to the pseudo-measured x and
   *                             y axes.
   */
  void update_depth(double z_depth, const Covariance1& R_depth,
                    double horizontal_variance = kDefaultHorizontalVariance);

  /// Error state eps = Local(estimate, X_true) in the filter's local chart
  /// (right/body multiplicative attitude, additive vectors), i.e. the chart
  /// covariance() lives in. Block order R, v, p, bg, ba.
  TangentVector errorStateVector(const MekfState& X_true) const;

  gtsam::Rot3 attitude() const;
  Vector3 velocity() const;
  Vector3 position() const;
  Vector3 bias_gyro() const;
  Vector3 bias_accel() const;

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
