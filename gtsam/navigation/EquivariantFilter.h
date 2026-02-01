/**
 * @file EquivariantFilter.h
 * @brief Equivariant Filter (EqF) implementation
 *
 * @author Darshan Rajasekaran
 * @author Jennifer Oum
 * @author Rohan Bansal
 * @author Frank Dellaert
 * @date 2025
 */

#pragma once

#include <gtsam/base/GroupAction.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ManifoldEKF.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>
#include <type_traits>

namespace gtsam {

/**
 * Equivariant Filter (EqF) for state estimation on Lie groups.
 *
 * The EqF estimates a Lie group state X ∈ G and a manifold state ξ ∈ M.
 * It uses a symmetry principle where the error dynamics are autonomous in a
 * specific frame.
 *
 * This implementation supports two modes:
 * 1. **Automatic**: The filter calculates Jacobian A using the input orbit.
 * 2. **Explicit**: You provide the Jacobian A and the manifold covariance Qc
 *    directly.
 *
 * @tparam M Manifold type for the physical state.
 * @tparam Symmetry Functor encoding the group action on the state.
 */
template <typename M, typename Symmetry>
class EquivariantFilter : public ManifoldEKF<M> {
 public:
  using Base = ManifoldEKF<M>;

  // Manifold traits
  static constexpr int DimM = Base::Dim;
  using TangentM = typename Base::TangentVector;
  using MatrixM = typename Base::Jacobian;
  using CovarianceM = typename Base::Covariance;

  // Group traits
  using G = typename Symmetry::Group;
  static constexpr int DimG = traits<G>::dimension;
  using TangentG = typename traits<G>::TangentVector;

  // Cross-dimension helpers
  using MatrixMG = Eigen::Matrix<double, DimM, DimG>;
  using MatrixGM = Eigen::Matrix<double, DimG, DimM>;

 private:
  M xi_ref_;  // Origin (reference) state on the manifold
  typename Symmetry::Orbit act_on_ref_;  // Orbit of the reference state
  MatrixMG Dphi0_;           // Differential of state action at identity
  MatrixGM InnovationLift_;  // Innovation lift matrix ((Dphi0)^+)

  G g_;  // Group element estimate

 public:
  /**
   * @brief Initialize the Equivariant Filter.
   *
   * @param xi_ref Reference manifold state (origin of lifted coordinates).
   * @param Sigma Initial covariance on the manifold.
   * @param X0 Initial group estimate (default: Identity).
   */
  EquivariantFilter(const M& xi_ref, const CovarianceM& Sigma,
                    const G& X0 = traits<G>::Identity())
      : Base(xi_ref, Sigma), xi_ref_(xi_ref), act_on_ref_(xi_ref), g_(X0) {
    // Compute differential of action phi at identity (Dphi0)
    act_on_ref_(traits<G>::Identity(), &Dphi0_);

    // Precompute the Innovation Lift matrix (pseudo-inverse of Dphi0)
    InnovationLift_ = Dphi0_.completeOrthogonalDecomposition().pseudoInverse();
    this->X_ = act_on_ref_(g_);
  }

  /// State on the manifold M is given by the base class
  using Base::state;

  /// errorCovariance that returns P_, on the equivariant filter error
  const typename Base::Covariance& errorCovariance() const { return this->P_; }

  /// Covariance in the tangent space at the current state.
  CovarianceM covariance() const {
    MatrixM J;
    if constexpr (MatrixM::RowsAtCompileTime == Eigen::Dynamic) {
      J.resize(this->n_, this->n_);
    }
    const typename Symmetry::Diffeomorphism action_at_g(g_);
    action_at_g(xi_ref_, &J);
    return J.transpose() * this->P_ * J;
  }

  /// @return Current group estimate.
  const G& groupEstimate() const { return g_; }

  /**
   * @brief Compute the error dynamics matrix A (Automatic).
   *
   * Calculates A = D_phi|_0 * D_lift|_u0, where u0 is the input mapped to the
   * origin.
   *
   * Concept requirements:
   * - `Lift` must be callable as `Lift(u_origin)(xi_ref, D_lift)` where
   *   D_lift is an OptionalJacobian of shape DimM x DimG.
   * - `InputOrbit` must be a group action on the input space with operator()
   *   that accepts the current group estimate X and returns the mapped input
   *   (no other methods are required by the filter).
   *
   * @tparam Lift Functor for the lift Λ(ξ, u).
   * @tparam InputOrbit Functor for the input orbit ψ_u.
   * @param psi_u Input Orbit instance.
   * @return MatrixM The calculated error dynamics matrix A.
   */
  template <typename Lift, typename InputOrbit>
  MatrixM computeErrorDynamicsMatrix(const InputOrbit& psi_u) const {
    MatrixGM D_lift;
    // Map current input to origin: u_origin = psi_u(X^-1)
    auto u_origin = psi_u(g_.inverse());

    // Lift at origin: D_lift = d(Lambda(xi_ref, u_origin))/dxi
    Lift lift_u_origin(u_origin);
    lift_u_origin(xi_ref_, &D_lift);

    return Dphi0_ * D_lift;
  }

  /**
   * @brief Discretize continuous-time error dynamics δ̇ = A δ over dt.
   *
   * On manifolds (unlike Lie groups) the error stays in a fixed tangent space
   * at the chosen origin, so discretization is just the matrix exponential of
   * A. K mirrors LieGroupEKF: K=1 gives Euler, K>1 calls expm(A*dt, K).
   */
  template <size_t K = 1>
  MatrixM transitionMatrix(const MatrixM& A, double dt) const {
    if constexpr (K == 1) {
      return this->I_ + A * dt;
    } else {
      return MatrixM(expm(A * dt, K));
    }
  }

  /**
   * @brief Propagate the filter state (Automatic).
   *
   * Automatically computes the error dynamics matrix A.
   *
   * Concept requirements:
   * - `Lift` is used as `Lift(u_origin)(xi_ref_, D_lift)` to obtain the lift
   *   and its Jacobian w.r.t. the manifold state.
   * - `InputOrbit` is only used via `psi_u(X_.inverse())` to map the current
   *   input to the origin; no other methods are needed.
   *
   * @tparam K Truncation order for discretization (1 = first order Euler,
   *         >1 uses matrix exponential expm(A*dt, K)).
   * @tparam Lift Functor for the lift Λ(ξ, u).
   * @tparam InputOrbit Functor for the input orbit ψ_u.
   * @param lift_u Lift functor for the current input.
   * @param psi_u Input Orbit for the current input.
   * @param A Error dynamics matrix (DimM x DimM).
   * @param Qc Process noise covariance on the manifold (continuous-time).
   * @param dt Time step.
   */
  template <size_t K = 1, typename Lift, typename InputOrbit>
  void predict(const Lift& lift_u, const InputOrbit& psi_u, const MatrixM& Qc,
               double dt) {
    // 1. Compute A automatically
    MatrixM A = computeErrorDynamicsMatrix<Lift>(psi_u);

    // 2. Delegate to explicit predict with manifold Qc
    predictWithJacobian<K>(lift_u, A, Qc, dt);
  }

  /**
   * @brief Propagate the filter state (Explicit).
   *
   * Uses provided Jacobian A and manifold covariance Qc. This allows `psi_u`
   * to be a pure Orbit without needing to implement `inputMatrixB`.
   *
   * Concept requirements:
   * - `Lift` is only used via `Lift(xi_est)` to produce a tangent vector.
   *   No additional methods are needed for this overload.
   *
   * @tparam Lift Functor for the lift Λ(ξ, u).
   * @param lift_u Lift functor for the current input.
   * @param A Error dynamics matrix (DimM x DimM).
   * @param Qc Process noise covariance on the manifold (continuous-time).
   * @param dt Time step.
   */
  template <size_t K = 1, typename Lift>
  void predictWithJacobian(const Lift& lift_u, const MatrixM& A,
                           const MatrixM& Qc, double dt) {
    // 1. Mean Propagation on Group
    M xi_est = this->state();          // Pure action
    TangentG Lambda = lift_u(xi_est);  // Pure lift

    g_ = traits<G>::Compose(g_, traits<G>::Expmap(Lambda * dt));
    M xi_next = act_on_ref_(g_);

    // 2. Covariance Propagation on Manifold
    MatrixM Phi = transitionMatrix<K>(A, dt);

    // Qc is manifold continuous-time covariance: Q_M = Qc * dt
    CovarianceM Q_manifold = Qc * dt;

    Base::predict(xi_next, Phi, Q_manifold);
  }

  /**
   * Measurement update: Corrects the state and covariance using a
   * pre-calculated predicted measurement and its Jacobian.
   *
   * Overwrites ManifoldEKF::update to modify g_ as well.
   *
   * @tparam Measurement type of the measurement space.
   * @param prediction Predicted measurement.
   * @param H Jacobian of the measurement function h.
   * @param z Observed measurement.
   * @param R Measurement noise covariance.
   */
  template <typename Measurement>
  void update(
      const Measurement& prediction,
      const Eigen::Matrix<double, traits<Measurement>::dimension, DimM>& H,
      const Measurement& z,
      const Eigen::Matrix<double, traits<Measurement>::dimension,
                          traits<Measurement>::dimension>& R) {
    static constexpr int MeasDim = traits<Measurement>::dimension;

    // Innovation: y = h(x_pred) - z. In tangent space: local(z, h(x_pred))
    // NOTE: we use the `z_hat - z` sign convention, NOT `z - z_hat`.
    typename traits<Measurement>::TangentVector innovation =
        traits<Measurement>::Local(z, prediction);

    // Kalman Gain: K = P H^T S^-1
    // K will be Eigen::Matrix<double, Dim, MeasDim>
    Eigen::Matrix<double, DimM, MeasDim> K = this->KalmanGain(H, R);

    // Correction in Manifold tangent space
    // K matches dimensions with innovation, so result is TangentM
    TangentM delta_xi = -K * innovation;

    // Lift correction to Group tangent space
    TangentG delta_x = InnovationLift_ * delta_xi;
    g_ = traits<G>::Compose(traits<G>::Expmap(delta_x), g_);
    this->X_ = act_on_ref_(g_);

    // Update covariance on Manifold using Joseph form
    this->JosephUpdate(K, H, R);
  }

  /// Same API as ManifoldEKF for measurement update with model function.
  template <typename Z, typename Func>
  void update(Func&& h, const Z& z,
              const Eigen::Matrix<double, traits<Z>::dimension,
                                  traits<Z>::dimension>& R) {
    static_assert(IsManifold<Z>::value,
                  "Template parameter Z must be a GTSAM Manifold.");

    Matrix H(traits<Z>::GetDimension(z), this->n_);
    Z prediction = h(this->X_, H);
    update<Z>(prediction, H, z, R);
  }

  /// Same API as ManifoldEKF for measurement update with vector inputs.
  void updateWithVector(const gtsam::Vector& prediction, const Matrix& H,
                        const gtsam::Vector& z, const Matrix& R) {
    this->validateInputs(prediction, H, z, R);
    update<Vector>(prediction, H, z, R);
  }
};

}  // namespace gtsam
