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
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ManifoldEKF.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

/**
 * Equivariant Filter (EqF) for state estimation on Lie groups.
 *
 * The EqF estimates a Lie group state X ∈ G and a manifold state ξ ∈ M.
 * It uses a symmetry principle where the error dynamics are autonomous in a
 * specific frame.
 *
 * Both ActionType::Right and ActionType::Left symmetries are supported. The
 * prediction increment acts at the current estimate, while the measurement
 * correction acts at the reference. Their composition sides depend on the action.
 *
 * Prediction comes in three forms:
 * 1. **Automatic**: predict() calculates the Jacobian A from the input orbit.
 * 2. **Explicit A**: predictWithJacobian() takes a continuous-time A and
 *    discretizes it.
 * 3. **Explicit transition**: predictWithTransition() takes an already
 *    discretized Phi and Qd, for models derived directly in discrete time.
 *
 * The filter propagates its error in **error coordinates**, the tangent space at
 * the reference state xi_ref: the covariance P, the error dynamics matrix A and
 * the process noise Qc all live there. Use covariance() to obtain the covariance
 * in the tangent space at the current state estimate, and actionDifferential()
 * for the map between the two frames.
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

 protected:

  /**
   * @brief Synchronization hooks for derived filters with dynamic runtime structure.
   *
   * Most fixed-dimension filters can rely on the public predict/update path only.
   * These functions are primarily for dynamic cases (for example variable-size
   * state/group representations) where derived classes need to resize/reset.
   */

  /// Reset reference, covariance, and group estimate; sync manifold state.
  void resetReferenceAndGroup(const M& xi_ref, const CovarianceM& P,
                              const G& g) {
    xi_ref_ = xi_ref;
    g_ = g;

    // Recompute Dphi0 and innovation lift matrix from current reference state
    act_on_ref_ = typename Symmetry::Orbit(xi_ref_);
    const G identity_at_g = traits<G>::Compose(g_.inverse(), g_);
    act_on_ref_(identity_at_g, &Dphi0_);
    InnovationLift_ = Dphi0_.completeOrthogonalDecomposition().pseudoInverse();

    if constexpr (DimM == Eigen::Dynamic) {
      this->n_ = traits<M>::GetDimension(xi_ref_);
      this->I_ = MatrixM::Identity(this->n_, this->n_);
    }
    this->P_ = P;
    this->X_ = act_on_ref_(g_);
  }

  /// Access current reference state used as the EqF chart origin.
  const M& referenceState() const { return xi_ref_; }

  /**
   * Evaluate the lift at the reference with input u_origin = psi_u(X^-1).
   * Used by automatic error linearization.
   */
  template <typename Lift, typename InputOrbit>
  TangentG liftAtOrigin(const InputOrbit& psi_u,
                        OptionalJacobian<DimG, DimM> D_lift = {}) const {
    Lift lift_u_origin(psi_u(g_.inverse()));
    return lift_u_origin(xi_ref_, D_lift);
  }

  /**
   * Advance using a lift evaluated at the current estimate and propagate
   * covariance with an already-discretized transition and process noise.
   * Commit g_ after the base class validates the matrix dimensions and updates
   * the manifold state and covariance.
   */
  void propagate(const TangentG& increment, const MatrixM& Phi,
                 const CovarianceM& Qd) {
    const G step = traits<G>::Expmap(increment);
    const G g_next = Symmetry::type == ActionType::Left
                         ? traits<G>::Compose(step, g_)
                         : traits<G>::Compose(g_, step);
    Base::predict(act_on_ref_(g_next), Phi, Qd);
    g_ = g_next;
  }

  /**
   * Apply an innovation correction, which lives in error coordinates at the
   * reference state, on the opposite side from the prediction increment.
   */
  void applyCorrection(const TangentG& delta_x) {
    const G step = traits<G>::Expmap(delta_x);
    g_ = Symmetry::type == ActionType::Left
             ? traits<G>::Compose(g_, step)
             : traits<G>::Compose(step, g_);
    this->X_ = act_on_ref_(g_);
  }

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
    const G identity_at_g = traits<G>::Compose(g_.inverse(), g_);
    act_on_ref_(identity_at_g, &Dphi0_);

    // Precompute the Innovation Lift matrix (pseudo-inverse of Dphi0)
    InnovationLift_ = Dphi0_.completeOrthogonalDecomposition().pseudoInverse();
    this->X_ = act_on_ref_(g_);
  }

  /// State on the manifold M is given by the base class
  using Base::state;

  /// errorCovariance that returns P_, on the equivariant filter error
  const typename Base::Covariance& errorCovariance() const { return this->P_; }

  /**
   * @brief Differential of the group action at the reference state.
   *
   * J = D phi_g|_{xi_ref} maps error coordinates, which live in the tangent
   * space at the reference state, to tangent vectors at the current state
   * estimate. The filter propagates its error in the former frame, so J is the
   * bridge to anything expressed at the current state.
   */
  MatrixM actionDifferential() const {
    MatrixM J;
    if constexpr (MatrixM::RowsAtCompileTime == Eigen::Dynamic) {
      J.resize(this->n_, this->n_);
    }
    const typename Symmetry::Diffeomorphism action_at_g(g_);
    action_at_g(xi_ref_, &J);
    return J;
  }

  /**
   * @brief Covariance in the tangent space at the current state.
   *
   * P_ is the covariance of the error coordinates in the tangent space at the
   * reference state xi_ref. The true state is recovered as xi = phi_g(e) with
   * e = Retract(xi_ref, epsilon), so a perturbation epsilon at the reference
   * appears at the current state as J * epsilon. The covariance therefore
   * pushes forward as J * P_ * J^T.
   */
  CovarianceM covariance() const {
    const MatrixM J = actionDifferential();
    return J * this->P_ * J.transpose();
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
   *   D_lift is an OptionalJacobian of shape DimG x DimM.
   * - `InputOrbit` must be a group action on the input space with operator()
   *   that accepts the current group estimate X and returns the mapped input
   *   (no other methods are required by the filter).
   *
   * @tparam Lift Functor for the lift Λ(ξ, u).
   * @tparam InputOrbit Functor for the input orbit ψ_u.
   * @param psi_u Input Orbit instance.
   * @return MatrixM The calculated error dynamics matrix A.
   *
   * The lift must reproduce the physical dynamics through the state action.
   * Automatic linearization additionally requires equivariance:
   * Lambda(phi_X(xi), psi_X(u)) = Ad_X Lambda(xi,u) for a left action,
   * or Ad_{X^-1} Lambda(xi,u) for a right action. These are separate
   * requirements, especially for actions with nontrivial stabilizers.
   * See the user guide doc/EquivariantFilter.ipynb for derivations
   * and examples. Explicit prediction accepts a caller-supplied error model
   * without requiring lift equivariance.
   */
  template <typename Lift, typename InputOrbit>
  MatrixM computeErrorDynamicsMatrix(const InputOrbit& psi_u) const {
    MatrixGM D_lift;
    liftAtOrigin<Lift>(psi_u, &D_lift);
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
   * Automatically computes the error dynamics matrix A. Requires the lift
   * equivariance condition documented in computeErrorDynamicsMatrix().
   * The supplied lift is also evaluated at the current estimate for the mean.
   *
   * Concept requirements:
   * - `Lift` is used as `Lift(u_origin)(xi_ref_, D_lift)` to obtain the lift
   *   and its Jacobian w.r.t. the manifold state.
   * - `InputOrbit` is only used via `psi_u(g_.inverse())` to map the current
   *   input to the origin; no other methods are needed.
   *
   * @tparam K Truncation order for discretization (1 = first order Euler,
   *         >1 uses matrix exponential expm(A*dt, K)).
   * @tparam Lift Functor for the lift Λ(ξ, u).
   * @tparam InputOrbit Functor for the input orbit ψ_u.
   * @param lift_u Lift functor for the current input.
   * @param psi_u Input Orbit for the current input.
   * @param Qc Process noise covariance on the manifold (continuous-time).
   * @param dt Time step.
   */
  template <size_t K = 1, typename Lift, typename InputOrbit>
  void predict(const Lift& lift_u, const InputOrbit& psi_u, const MatrixM& Qc,
               double dt) {
    const MatrixM A = computeErrorDynamicsMatrix<Lift>(psi_u);
    predictWithJacobian<K>(lift_u, A, Qc, dt);
  }

  /**
   * @brief Propagate the filter state (Explicit).
   *
   * Uses the provided error Jacobian A and process covariance Qc.
   *
   * Concept requirements:
   * - `Lift` is only used via `Lift(xi_est)` to produce a tangent vector.
   *   No additional methods are needed for this overload.
   *
   * The lift is evaluated at the current estimate and must generate the
   * physical dynamics through the declared state action. Prediction composes
   * Exp(Lambda dt) on the left for a left action and on the right for a right
   * action. Equivariance and an input orbit are not required when A is supplied.
   * A and Qc must be expressed in error coordinates at the reference state.
   * Left-action callers that previously supplied body-frame increments for
   * right multiplication must convert their lift to the declared action.
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
    const TangentG Lambda = lift_u(this->state());
    propagate(Lambda * dt, transitionMatrix<K>(A, dt), CovarianceM(Qc * dt));
  }

  /**
   * @brief Propagate with an already-discretized transition and process noise.
   *
   * For models derived directly in discrete time, or where a closed-form
   * exp(A dt) is available, this avoids the truncated-series discretization of
   * transitionMatrix(), which is only first order at the default K = 1.
   *
   * The lift has the same contract as predictWithJacobian(): it is evaluated
   * at the current estimate and generates motion through the state action.
   * No input orbit or equivariance condition is required.
   *
   * @param lift_u Lift functor for the current input.
   * @param Phi Discrete transition matrix over dt (DimM x DimM).
   * @param Qd Discrete process noise over dt, in error coordinates.
   * @param dt Time step, used for the mean only.
   */
  template <typename Lift>
  void predictWithTransition(const Lift& lift_u, const MatrixM& Phi,
                             const CovarianceM& Qd, double dt) {
    propagate(lift_u(this->state()) * dt, Phi, Qd);
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
    applyCorrection(delta_x);

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

  /// Vector measurement update using a custom innovation lift delta_x=f(delta_xi).
  template <typename InnovationLiftFn>
  void updateWithVector(const gtsam::Vector& prediction, const Matrix& H,
                        const gtsam::Vector& z, const Matrix& R,
                        InnovationLiftFn&& innovationLift) {
    this->validateInputs(prediction, H, z, R);

    const gtsam::Vector innovation = traits<Vector>::Local(z, prediction);
    const Matrix K = this->KalmanGain(H, R);
    const TangentM delta_xi = -K * innovation;
    const TangentG delta_x = innovationLift(delta_xi);

    applyCorrection(delta_x);
    this->JosephUpdate(K, H, R);
  }
};

}  // namespace gtsam
