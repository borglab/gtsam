#pragma once
#include <gtsam/navigation/InvariantEKF.h>
#include <gtsam_unstable/tfg_inekf/TFGGeometry.h>

#include <Eigen/Dense>

// TFG-InEKF: biased INS on G_TF via InvariantEKF<TwoFrameGroup>. The process
// model (lift) and measurement outputs live here; the group and its action
// are in TFGGeometry.h.

namespace tfg {

/// Lifted IMU input for InvariantEKF::predict (biases are in the state).
struct ImuInput {
  Vector3 omega;
  Vector3 accel;
  Vector3 tau_omega = Vector3::Zero();
  Vector3 tau_accel = Vector3::Zero();

  /// Gravity vector in the global frame {G}. The filter is frame-agnostic: the
  /// caller's choice of sign is what fixes which way +z points (Z-down/NED
  /// gives +g on z, Z-up/ENU gives -g). Defaults to zero, i.e. no gravity.
  Vector3 g_vec = Vector3::Zero();
};

/// Continuous-time IMU noise PSDs, per-axis (diagonal, not isotropic).
struct ImuNoise {
  Vector3 gyro = Vector3::Zero();
  Vector3 accel = Vector3::Zero();
  Vector3 gyro_rw = Vector3::Zero();
  Vector3 accel_rw = Vector3::Zero();
};

/// Equivariant lift Lambda(X, u).
Tangent lift(const TwoFrameGroup& X, const ImuInput& u);

/// Df = d lift(X * Expmap(eps), u) / d eps at eps = 0 (bias->navigation
/// coupling).
TwoFrameGroup::Jacobian liftJacobian(const TwoFrameGroup& X, const ImuInput& u);

/// Qd = B diag(sigma^2) B^T dt; B maps (n_omega, n_accel, tau_w, tau_a) into
/// lift. Non-zero blocks: B[theta,n_w]=I, B[v,n_a]=I, B[g_w,n_w]=b_w^,
/// B[g_w,tau_w]=-I, B[g_a,n_w]=b_a^, B[g_a,tau_a]=-I.
TwoFrameGroup::Jacobian inputNoiseCov(const TwoFrameGroup& X,
                                      const ImuNoise& noise, double dt);

/// Group increment U = Expmap(lift(X,u) * dt) for InvariantEKF::predict.
TwoFrameGroup increment(const TwoFrameGroup& X, const ImuInput& u, double dt);

/// DVL body-frame velocity output (natively equivariant).
struct BodyVelocityOutput {
  static constexpr int dim = 3;
  using Jacobian = Eigen::Matrix<double, 3, 15>;

  /// Predicted body-frame velocity h_d = R^T v.
  static Vector3 predict(const TwoFrameGroup& xi);

  /// Output action psi_d(X, y) = R_X^T y + R_X^T v_X; for equivariance checks.
  static Vector3 output_action(const TwoFrameGroup& X, const Vector3& y);

  /// Output Jacobian H = [ y_hat^  I  0  0 ] in the right-retract chart.
  /// d_v block is identity (velocity perturbed in rotated frame, not R^T).
  static Jacobian jacobian(const TwoFrameGroup& xi_hat);
};

/// Position output (equivariant body-frame residual).
struct PositionOutput {
  static constexpr int dim = 3;
  using Jacobian = Eigen::Matrix<double, 3, 15>;

  enum class Variant {
    C0,     ///< first-order Jacobian
    Cstar,  ///< midpoint-symmetrised (cubic linearisation error)
  };

  /// Body-frame residual h = R^T (pi - p).
  static Vector3 predict(const TwoFrameGroup& xi, const Vector3& pi);

  /// Output action rho_X(y) = R_X^T (y - p_X); for equivariance checks.
  static Vector3 output_action(const TwoFrameGroup& X, const Vector3& y);

  /// Output Jacobian for chart xi(eps) = xi_hat * Expmap(eps).
  /// Right-retract chart: noise-free residual y_hat = J_l(eps_theta) eps_p.
  ///   C0    = [ y_hat^      0  -I  0 ]
  ///   Cstar = [ 1/2 y_hat^  0  -I  0 ]  (halves attitude coupling ->
  ///   O(||e||^3) error)
  static Jacobian jacobian(const TwoFrameGroup& xi_hat, const Vector3& pi,
                           Variant variant = Variant::C0);
};

class TfgInEKF : public gtsam::InvariantEKF<TwoFrameGroup> {
 public:
  using Base = gtsam::InvariantEKF<TwoFrameGroup>;
  using Covariance = Eigen::Matrix<double, 15, 15>;
  using Covariance3 = Eigen::Matrix<double, 3, 3>;
  using Covariance1 = Eigen::Matrix<double, 1, 1>;

  /// Default variance (m^2) given to the pseudo-measured horizontal axes of a
  /// depth update, large enough that the update leaves x and y untouched.
  static constexpr double kDefaultHorizontalVariance = 1e3;

  /// @param X0 initial group state (TwoFrameGroup::FromState for physical
  /// coords)
  /// @param P0 initial 15-dim tangent covariance.
  TfgInEKF(const TwoFrameGroup& X0, const Covariance& P0);

  /// IMU predict with lifted tangent process noise Qc (scaled by dt).
  /// @param g_vec  Global-frame gravity vector; its sign fixes the frame
  ///               convention (Z-down/NED: +g on z).
  void propagate(const Vector3& omega, const Vector3& accel,
                 const Vector3& g_vec, const Covariance& Qc, double dt);

  /// IMU predict; builds Qd = B diag(sigma^2) B^T dt from ImuNoise PSDs.
  void propagate(const Vector3& omega, const Vector3& accel,
                 const Vector3& g_vec, const ImuNoise& noise, double dt);

  /// Position update: h = R^T(pi - p).
  void update_position(const Vector3& pi, const Covariance3& R_pos,
                       bool use_cstar = false);

  /// DVL update: h_d = R^T v; R_dvl in body frame.
  void update_dvl(const Vector3& z_dvl, const Covariance3& R_dvl);

  /**
   * Pressure-sensor depth update from the world-frame z position.
   *
   * The scalar model h(xi) = e_3^T p is not equivariant, so the depth is
   * stacked onto the estimated horizontal position to form the pseudo-position
   * p_tilde = [p_x, p_y, z_depth]^T, which is fed through the position output
   * with the horizontal variance inflated so only the vertical direction is
   * corrected. Since the horizontal entries come from the estimate rather than
   * the true state, this update is a second-order approximation. See Sec. V-B
   * of tg_eqf/docs/EqF_design_for_DVL_Depth_aided_INS.pdf.
   *
   * @param z_depth              Measured world-frame z position.
   * @param R_depth              Variance of z_depth, as a 1x1 matrix.
   * @param horizontal_variance  Variance assigned to the pseudo-measured x and
   *                             y axes.
   * @param use_cstar            Forwarded to update_position.
   */
  void update_depth(double z_depth, const Covariance1& R_depth,
                    double horizontal_variance = kDefaultHorizontalVariance,
                    bool use_cstar = false);

  /// Error state eps = Local(estimate, X_true) in the group's local chart, i.e.
  /// the chart covariance() lives in. Block order R, v, p, bg, ba.
  TangentVector errorStateVector(const TwoFrameGroup& X_true) const;

  // State accessors
  gtsam::Rot3 attitude() const;
  Vector3 velocity() const;
  Vector3 position() const;
  Vector3 bias_gyro() const;
  Vector3 bias_accel() const;
};

}  // namespace tfg
