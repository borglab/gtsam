#pragma once
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/navigation/EquivariantFilter.h>
#include <gtsam_unstable/tg_eqf/TGGeometry.h>

#include <Eigen/Dense>

namespace gtsam {
namespace tgeqf {

/// Extended input u in R^21.
struct Input {
  Vector3 w = Vector3::Zero();
  Vector3 a = Vector3::Zero();
  Vector3 v = Vector3::Zero();  // virtual input \nu
  Vector3 tau_w = Vector3::Zero();
  Vector3 tau_a = Vector3::Zero();
  Vector3 tau_v = Vector3::Zero();
  Vector3 g_vec = Vector3::Zero();

  /// Coefficient vector of the extended input.
  using Vector21 = Eigen::Matrix<double, 21, 1>;

  Vector21 vector() const;
  static Input from_vector(const Vector21& v);
};

/**
 * Lift functor Lambda(xi, u) : M x L -> g.
 *
 * Closed form from expanding T^{-1}(G-N)T (the N/-N homogeneous-row coupling
 * cancels exactly, leaving only R^T acting on g and v):
 *
 *   Lambda_1.w = w - b_w
 *   Lambda_1.a = a - b_a + R^T g
 *   Lambda_1.v = nu - b_v + R^T v
 *   Lambda_2   = ad_b[Lambda_1] - tau
 */
struct Lift {
  Input u;

  explicit Lift(const Input& u);

  /// Lambda(xi, u) with optional Jacobian d(Lambda)/d(xi) in R^{18 x 18}.
  State::TangentVector operator()(const State& xi,
                                  OptionalJacobian<18, 18> D_lift = {}) const;
};

/// Input orbit psi_u : G -> L.
struct InputOrbit {
  Input u;

  explicit InputOrbit(const Input& u);

  Input operator()(const TGElement& X) const;
};

/**
 * Equivariant DVL body-frame velocity measurement h_d(xi) = R^T v.
 *
 * Jacobians below are already in error coordinates at the reference state --
 * pass to TGEqF::updateWithReset() unchanged. The measurement arrives in the
 * true state's body frame; pull it back with inverse_output_action() first.
 */
struct DVLMeasurement {
  /// Output matrix in error coordinates, d(output)/d(eps) at eps = 0.
  using Jacobian = Eigen::Matrix<double, 3, 18>;

  static Vector3 predict(const State& xi);

  /**
   * Origin-chart Jacobian, C0 = [ y0^  I_3  0  0 ], y0 = R0^T v0.
   *
   * The output differential at the origin output alone: second order in the
   * state error. C* below is what the filter actually uses; C0 is the
   * baseline the accuracy tests compare it against.
   */
  static Jacobian jacobian_C0(const State& xi_ref);

  /**
   * Equivariant output matrix, C* = [ 0.5 (y0 + y_tilde)^  I_3  0  0 ], with
   * y0 = R0^T v0 and y_tilde = psi_X^{-1}(z) (the measurement pulled back to
   * the origin output space).
   *
   * Taking the differential at the midpoint of y0 and y_tilde instead of at
   * y0 alone makes the linearization error third order rather than second --
   * this is the matrix the filter uses.
   */
  static Jacobian jacobian_Cstar(const State& xi_ref, const TGElement& g,
                                 const Vector3& z_dvl);

  /// Output group action psi_X(y) = R_X^T y + R_X^T v_X.
  static Vector3 output_action(const TGElement& X, const Vector3& y);

  /// Inverse output action psi_X^{-1}(y) = R_X y - v_X; carries a measurement
  /// from the true state's body frame to the origin output space.
  static Vector3 inverse_output_action(const TGElement& X, const Vector3& y);
};

/**
 * Equivariant global position measurement h'(xi) = R^T(pi - p). Used for a
 * direct position fix (e.g. USBL) and, with the estimated horizontal
 * position, as the pseudo-position stand-in for the depth update (see
 * TGEqF::update_depth).
 *
 * Jacobians below are already in error coordinates at the reference state --
 * pass to TGEqF::updateWithReset() unchanged.
 */
struct PositionMeasurement {
  /// Output matrix in error coordinates, d(output)/d(eps) at eps = 0.
  using Jacobian = Eigen::Matrix<double, 3, 18>;

  /// General single-state residual R^T(pi - p).
  static Vector3 predict(const State& xi, const Vector3& pi);

  /// y0 - g.p, with y0 = R0^T(pi - p0); equals R0^T(pi - p_hat) since the
  /// estimate satisfies p_hat = R0 * g.p + p0.
  static Vector3 predictAtOrigin(const State& xi_ref, const TGElement& g,
                                 const Vector3& pi);

  /// Origin-chart Jacobian, C0 = [ y0^  0  -I_3  0 ], y0 = R0^T(pi - p0). Same
  /// rationale as DVLMeasurement::jacobian_C0, reading p instead of v.
  static Jacobian jacobian_C0(const State& xi_ref, const Vector3& pi);

  /// Equivariant output matrix, C* = [ 0.5 (y0 + p_X)^  0  -I_3  0 ], with
  /// p_X = g.p the transported measurement. Same rationale as
  /// DVLMeasurement::jacobian_Cstar, reading p instead of v.
  static Jacobian jacobian_Cstar(const State& xi_ref, const TGElement& g,
                                 const Vector3& pi);

  /// Output group action psi_X(y) = R_X^T (y - p_X).
  static Vector3 output_action(const TGElement& X, const Vector3& y);
};

/**
 * Non-equivariant pressure-sensor depth output h(xi) = e_3^T p.
 *
 * p -> p + R p_A under the state action, so h(phi(X,xi)) = h(xi) + e_3^T R p_A
 * depends on the unmeasured rotation: no output action can reproduce it,
 * hence no C* (and no output_action/predictAtOrigin) here.
 */
struct DepthMeasurement {
  /// Output matrix in error coordinates, d(output)/d(eps) at eps = 0.
  using Jacobian = Eigen::Matrix<double, 1, 18>;

  /// Vertical position e_3^T p.
  static double predict(const State& xi);

  /**
   * C = [ (g.p x n)^T  0  n^T  0 ], n = R0^T e_3: the exact chart derivative
   * of eps -> h(phi(g, Retract(xi_ref, eps))) at eps = 0.
   *
   * Retract is phi(Expmap_G(eps), xi_ref), and phi is a right action, so the
   * composed map is h(phi(Expmap_G(eps) g, xi_ref)); its position slot is
   * R0 (Exp_SO3(eps_R) g.p + J_l(eps_R) eps_p) + p0, giving the two blocks.
   */
  static Jacobian jacobian_C(const State& xi_ref, const TGElement& g);
};

/**
 * Continuous-time IMU noise PSDs, mapped through the lift differential B
 * into process noise (see inputNoiseCov()). Per-axis, not isotropic.
 *
 * No channel for the virtual input nu (held at zero, not measured) or its
 * bias b_v (driven only by whatever the lift couples into it from the
 * gyro/accel channels below).
 */
struct ImuNoise {
  Vector3 gyro = Vector3::Zero();
  Vector3 accel = Vector3::Zero();
  Vector3 gyro_rw = Vector3::Zero();
  Vector3 accel_rw = Vector3::Zero();
};

/**
 * TG-EqF biased INS filter on G = SE_2(3) ⋉ se_2(3).
 *
 * The base class keeps every matrix in **error coordinates**, the tangent space
 * at the fixed reference state xi_ref, so every measurement here supplies an
 * output matrix C* built at that reference state and hands it to
 * updateWithReset().
 */
class TGEqF : public gtsam::EquivariantFilter<State, TGSymmetry> {
 public:
  using Base = gtsam::EquivariantFilter<State, TGSymmetry>;
  using Covariance18 = Eigen::Matrix<double, 18, 18>;
  using Covariance15 = Eigen::Matrix<double, 15, 15>;
  using Covariance3 = Eigen::Matrix<double, 3, 3>;
  using Covariance1 = Eigen::Matrix<double, 1, 1>;

  /// Default variance (m^2) given to the pseudo-measured horizontal axes of a
  /// depth update, large enough that the update leaves x and y untouched.
  static constexpr double kDefaultHorizontalVariance = 1e6;

  /// Initial standard deviation of the virtual bias b_v.
  static constexpr double kVirtualBiasInitialSigma = 1e-6;

  /**
   * Initial covariance built from the physical-state block [R,v,p,b_w,b_a];
   * the virtual-bias block is kVirtualBiasInitialSigma^2 * I, uncorrelated.
   *
   * v and p are read in State's chart (SE_2(3) logarithm coordinates), so
   * they're plain global-frame variances only at identity rotation --
   * otherwise they're resolved in that state's body frame and couple to the
   * rotation block.
   */
  static Covariance18 initialCovariance(const Covariance15& Sigma_physical);

  /**
   * Construct at the identity origin (xi_ref = identity). This is the
   * interface examples should use, matching TfgInEKF and MEKF.
   *
   * @param X0  Initial group estimate; state is X0.
   * @param P0  Initial covariance of X0, expressed in the tangent chart at X0.
   */
  explicit TGEqF(const TGElement& X0, const Covariance18& P0);

  /**
   * Construct at an arbitrary reference state xi_ref. Exposed for testing;
   * examples should use the identity-origin constructor above.
   *
   * @param xi_ref  Fixed origin state.
   * @param P0      Initial covariance of the initial estimate phi(X0, xi_ref),
   *                expressed in the tangent chart at that estimate. It is
   *                transported internally to the origin chart at xi_ref, so
   *                covariance() returns Sigma0 unchanged right after
   *                construction. The transport is the identity when X0 is the
   *                identity.
   * @param X0      Initial group estimate; state is phi(X0, xi_ref).
   */
  explicit TGEqF(const State& xi_ref, const Covariance18& P0,
                 const TGElement& X0 = TGElement::Identity());

  // Enable/disable covariance reset after each update.
  void set_reset_step(bool enable) { reset_step_ = enable; }

  /// DVL Jacobian update_dvl() uses. Cstar (default) is the equivariant,
  /// third-order matrix the filter is designed around; C0 is the naive,
  /// second-order chart derivative, kept for ablation comparisons.
  enum class DvlJacobian { Cstar, C0 };
  void set_dvl_jacobian(DvlJacobian mode) { dvl_jacobian_ = mode; }

  /// Reset transport J(delta_xi, delta_x); P <- J P J^T. Exposed for testing.
  MatrixM resetMatrix(const TangentVector& delta_xi,
                      const TangentVector& delta_x) const;

  /**
   * IMU propagation with a continuous-time origin-chart process noise density
   * Qc, discretized by Van Loan as Qd = int_0^dt exp(A s) Qc exp(A^T s) ds.
   * Use the ImuNoise overload for lift-mapped process noise instead.
   */
  void propagate(const Vector3& w_meas, const Vector3& a_meas,
                 const Vector3& g_vec, const Covariance18& Qc, double dt);

  /**
   * IMU propagation with Qc = B Sigma B^T from IMU PSDs through the lift
   * differential (origin chart).
   */
  void propagate(const Vector3& w_meas, const Vector3& a_meas,
                 const Vector3& g_vec, const ImuNoise& noise, double dt);

  // Qc_eff = B Sigma B^T, the continuous-time density; propagate() applies
  // the Van Loan discretization. For testing.
  Covariance18 inputNoiseCov(const ImuNoise& noise) const;

  /// Fixed-origin input lift orbit_jacobian0_ * G; reduces to G itself since
  /// orbit_jacobian0_ is always I_18. For testing.
  MatrixM input_lift() const { return input_lift_; }

  // DVL body-velocity update via h(xi) = R^T v, using the output matrix
  // selected by set_dvl_jacobian() (the equivariant C* by default).
  void update_dvl(const Vector3& z_dvl, const Covariance3& R_dvl);

  // Position update via h'(xi) = R^T(pi - p), using the equivariant C*.
  void update_position(const Vector3& pi, const Covariance3& R_pos);

  /**
   * Pressure-sensor depth update from the world-frame z position using the
   * right error pseudo position output.
   *
   * @param z_depth              Measured world-frame z position.
   * @param R_depth              Variance of z_depth, as a 1x1 matrix.
   * @param horizontal_variance  Variance assigned to the pseudo-measured x and
   *                             y axes.
   */
  void update_depth(double z_depth, const Covariance1& R_depth,
                    double horizontal_variance = kDefaultHorizontalVariance);

  /**
   * Pressure-sensor depth update through the non-equivariant scalar model
   * h(xi) = e_3^T p (see DepthMeasurement above).
   *
   * @param z_depth  Measured world-frame z position.
   * @param R_depth  Variance of z_depth, as a 1x1 matrix.
   */
  void update_depth_direct(double z_depth, const Covariance1& R_depth);

  // Error state and vector in the fixed-origin chart.
  State errorState(const State& xi_true) const;
  TangentVector errorStateVector(const State& xi_true) const;

  // State accessors.
  gtsam::Rot3 attitude() const;
  Vector3 velocity() const;
  Vector3 position() const;
  Vector3 bias_gyro() const;
  Vector3 bias_accel() const;
  Vector3 bias_vel() const;

  ExtendedPose3<2> extended_pose() const;
  Vector9 biases() const;

 private:
  // Compute the fixed-origin caches (orbit_jacobian0_, innovation_lift_,
  // input_lift_) from the current referenceState(). Called on construction.
  void computeOriginCaches();

  /// Measurement update followed by the covariance reset.
  void updateWithReset(const Vector& prediction, const Matrix& Cstar,
                       const Vector& z, const Matrix& R);

  bool reset_step_ = true;
  DvlJacobian dvl_jacobian_ = DvlJacobian::Cstar;

  // Fixed-origin caches for the current reference state (xi_ref_).
  MatrixM innovation_lift_;
  MatrixM orbit_jacobian0_;
  MatrixM input_lift_;
};

}  // namespace tgeqf
}  // namespace gtsam
