/**
 * @file testEquivariantFilter.cpp
 * @brief Simple SO(3) equivariant filter example (attitude-only),
 *        exercising EquivariantFilter with a different M/G/Actions combo.
 *
 * This is inspired by the simple sphere / attitude example in Mahony's
 * equivariant filter tutorial, but here formulated for S^2 directions:
 * - Physical state M is Unit3 (a direction on S^2).
 * - Symmetry group G is Rot3 (attitude).
 * - The state estimate is recovered as \hat{η} = Q^T \bar{η}, matching Mahony's
 * notation.
 *
 * The goal is to ensure EquivariantFilter.h is generic and not tied to ABC.h.
 *
 * The innovation term follows Mahony's equivariant update on S², where the
 * error is defined via the right action φ_{η̄}(Q) = Qᵀη̄ and innovations are
 * formed from ρ_y(Q̂⁻¹).
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/GroupAction.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/EquivariantFilter.h>

using namespace gtsam;

namespace attitude_example {

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

using M = Unit3;  // physical state: direction η on S^2
using G = Rot3;   // symmetry group: SO(3) attitude Q

//---------------------------------------------------------------------------
// Symmetry: group action on the state
//   φ_η(Q) = Q^T η.
//---------------------------------------------------------------------------

struct Symmetry : public GroupAction<Symmetry, G, M> {
  static constexpr ActionType type = ActionType::Right;

  /// Group action at state eta by group element Q.
  M operator()(const M& eta, const G& Q, OptionalJacobian<2, 2> H_eta = {},
               OptionalJacobian<2, 3> H_Q = {}) const {
    // Apply the right action Q^T * eta as in Mahony's example.
    return Q.unrotate(eta, H_Q, H_eta);
  }
};

//---------------------------------------------------------------------------
// Dynamics
//------------------------------------------- --------------------------------

// Compute the dynamics at a given point on the manifold
Vector2 dynamics(const Vector3& omega, const M& eta) {
  // eta.cross(omega_) is in 3D ambient space
  const Vector3 ambient_dynamics = eta.cross(omega);

  // Convert to 2D tangent space via local basis
  return eta.basis().transpose() * ambient_dynamics;
}

//---------------------------------------------------------------------------
// Lift: maps (state, input) -> group tangent
//
// For this simple example, the lift just returns the body angular velocity
// itself, independent of the state. This is enough to test the EqF plumbing.
//---------------------------------------------------------------------------

struct Lift {
  explicit Lift(const Vector3& omega) : omega_(omega) {}

  typename traits<G>::TangentVector operator()(
      const M& /*R*/, OptionalJacobian<3, 2> H = {}) const {
    if (H) *H = Matrix::Zero(3, 2);
    // Tangent space of SO(3) ~ R^3, so we simply return omega.
    return omega_;
  }

 private:
  Vector3 omega_;
};

//---------------------------------------------------------------------------
// Input action: right action on the input omega.
//---------------------------------------------------------------------------

struct InputAction : public GroupAction<InputAction, G, Vector3> {
  static constexpr ActionType type = ActionType::Right;

  /// Right group action on input: Q^{-1} * omega.
  Vector3 operator()(const Vector3& omega, const G& Q,
                     OptionalJacobian<3, 3> H_omega = {},
                     OptionalJacobian<3, 3> H_Q = {}) const {
    return Q.unrotate(omega, H_Q, H_omega);
  }
};

// Use the generated Orbit type directly for the pure group action.
using InputOrbit = InputAction::Orbit;

/// Embed process noise covariance into the lifted coordinates (identity for
/// this simple example).
inline Matrix3 processNoise(const Matrix3& Sigma) { return Sigma; }

/// Derivative of the lifted dynamics wrt. local coordinates.
inline Matrix2 stateMatrixA(const G& /*Q_hat*/) { return Matrix2::Zero(); }

/// Input matrix B that maps process noise to manifold coordinates.
inline Matrix23 inputMatrixB(const G& /*Q_hat*/) {
  // TODO(Frank): suspect, should depend on eta? Or make sure it works for
  // chosen reference direction.
  return Matrix23::Identity();
}

struct MeasurementFunctor {
  double c_m_;
  explicit MeasurementFunctor(double c_m) : c_m_(c_m) {}

  /// Measurement function h(η̂) = c_m * η̂.
  Vector3 operator()(const Unit3& eta_hat,
                     OptionalJacobian<3, 2> H = {}) const {
    if (H) {
      *H = c_m_ * eta_hat.basis();
    }
    return c_m_ * eta_hat.point3(H);
  }
};

//---------------------------------------------------------------------------
// example values
//---------------------------------------------------------------------------

const G Q0;  // Rot3() == identity (observer state \hat{Q})

const M eta_ref = Unit3(0, 0, 1);  // \bar{η}
const typename Symmetry::Orbit phi_ref(
    eta_ref);  // state action on reference direction
const double c_m = 1.2;

// Test dynamics on manifold: eta x omega
Rot3 Q1 = Rot3::Ypr(0.1, 0.2, 0.3);   // non-identity rotation
Unit3 eta1 = Q1.inverse() * eta_ref;  // eta1 = Q1 * eta_ref

const Vector3 omega(0.1, -0.2, 0.3);
Lift lift_omega(omega);

const InputOrbit psi_u(omega);
}  // namespace attitude_example

//==============================================================================
// Symmetry Tests
//==============================================================================
TEST(EquivariantFilter_Attitude, Symmetry) {
  using namespace attitude_example;

  // Check the state action
  Unit3 expected_eta1 = phi_ref(Q1);
  EXPECT(assert_equal(expected_eta1, eta1));

  // Check jacobian at identity
  Matrix Dphi0;
  phi_ref(Q0, Dphi0);

  // Expected Jacobian
  Matrix expected_Dphi0;
  Q0.unrotate(eta_ref, expected_Dphi0);

  EXPECT(assert_equal(expected_Dphi0, Dphi0));
}

//==============================================================================
// Test the induced group action
TEST(EquivariantFilter_Attitude, InducedGroupAction) {
  using namespace attitude_example;

  // Purposefully a vector field which is not the dynamics
  const auto f = [](const Unit3& eta) -> Vector2 {
    auto p = eta.point3();
    return Vector2(p.z() * p.x() * 3, p.y() * p.x());
  };

  using InducedField = Symmetry::InducedVectorField<decltype(f)>;
  const InducedField fInduced(Q1, f);

  // A bit of a self-fulfilling prophecy:
  Matrix H;
  Symmetry::Diffeomorphism{Q1}(Symmetry::Orbit{eta1}(Q1.inverse()), H);
  const Vector2 expected = H * f(Symmetry::Orbit{eta1}(Q1.inverse()));
  const Vector2 actual = fInduced(eta1);
  EXPECT(assert_equal(expected, actual));
}

//==============================================================================
TEST(EquivariantFilter_Attitude, DiffeomorphismPushForward) {
  using namespace attitude_example;

  const Vector2 tangent = (Vector2() << 0.1, -0.2).finished();
  Matrix H;
  Symmetry::Diffeomorphism phi_Q1(Q1);
  phi_Q1(eta1, H);
  const Vector2 expected = H * tangent;
  const Vector2 actual = phi_Q1.pushforward(eta1, tangent);
  EXPECT(assert_equal(expected, actual));
}

//==============================================================================
// Dynamics Tests
//==============================================================================
// Manifold dynamics at reference direction (north pole)
TEST(EquivariantFilter_Attitude, Dynamics0) {
  using namespace attitude_example;

  Vector3 omega_y(0, 0.1, 0);

  // The tangent space basis for Unit3(0,0,1) is the Y and -X axis.
  // std::cout << eta_ref.basis() << std::endl; // comment out to verify

  // Manually calculate expected dynamics at eta_ref (Unit3(0,0,1))
  // eta_ref.cross(omega_y) = (0,0,1) x (0, 0.1, 0) = (-0.1, 0, 0)
  // The projection of (-0.1, 0, 0) onto tangent space will be (0, 0.1).
  Vector2 expected_dynamics = Vector2(0, 0.1);

  Vector2 actual_dynamics = dynamics(omega_y, eta_ref);
  EXPECT(assert_equal(expected_dynamics, actual_dynamics));
}

//==============================================================================
// Regression test and affine map construction
TEST(EquivariantFilter_Attitude, Dynamics) {
  using namespace attitude_example;

  // Manually calculate expected dynamics: eta1.cross(omega)
  Matrix23 B1t = eta1.basis().transpose();
  Vector2 expected = B1t * eta1.cross(omega);
  Vector2 actual = dynamics(omega, eta1);
  EXPECT(assert_equal(expected, actual));

  // Regression test
  EXPECT(assert_equal(Vector2(0.143203, -0.279723), actual, 1e-5));

  // Define vector fields f_i(eta) = eta.cross(e_i)
  Vector2 f1 = B1t * eta1.cross(Point3(1, 0, 0));
  Vector2 f2 = B1t * eta1.cross(Point3(0, 1, 0));
  Vector2 f3 = B1t * eta1.cross(Point3(0, 0, 1));

  // Dynamics as an affine map: omega_x * f1 + omega_y * f2 + omega_z * f3
  Vector2 actual_affine_map = omega.x() * f1 + omega.y() * f2 + omega.z() * f3;
  EXPECT(assert_equal(actual, actual_affine_map));
}

//==============================================================================
TEST(EquivariantFilter_Attitude, InputEquivariance) {
  using namespace attitude_example;

  // Use knowledge that induced dynamics is eta .cross(Q.inverse() * omega)
  const Vector2 expected =
      eta1.basis().transpose() * eta1.cross(Q1.inverse() * omega);
  const Vector2 equivariant =
      dynamics(psi_u(Q1), eta1);  // dynamics given input transformed omega
  EXPECT(assert_equal(expected, equivariant));

  // f_u is dynamics given input omega
  auto f_u = [&](const M& eta) -> Vector2 { return dynamics(omega, eta); };
  using InducedField = Symmetry::InducedVectorField<decltype(f_u)>;
  const InducedField fInduced(Q1, f_u);

  const Vector2 induced = fInduced(eta1);
  EXPECT(assert_equal(expected, induced));
  // Equation (3.5) in Fornasier thesis
  EXPECT(assert_equal(induced, equivariant));
}

//==============================================================================
// Lift shadow property: Dφ_{η}(I) Λ(η, u) = ξ̇ on the manifold.
TEST(EquivariantFilter_Attitude, LiftShadowManifoldDynamics) {
  using namespace attitude_example;

  Symmetry::Orbit phi_eta(eta1);
  Matrix H;
  phi_eta(G::Identity(), H);  // derivative w.r.t. group at identity

  const Vector3 lifted = Lift(omega)(eta1);
  const Vector2 shadow = H * lifted;

  const Vector2 manifold = dynamics(omega, eta1);
  EXPECT(assert_equal(shadow, manifold));
}

//==============================================================================
TEST(EquivariantFilter_Attitude, LiftEquivariance) {
  using namespace attitude_example;

  // Λ should satisfy Ad_{Q^{-1}} Λ(η, u) = Λ(φ_Q(η), ψ_Q(u)).
  Lift lift_omega(omega);
  InputOrbit psi_u(omega);

  const Rot3 g = Q1;
  const Vector3 lifted = lift_omega(eta1);
  const Vector3 lifted_ad = g.inverse().AdjointMap() * lifted;

  const Vector3 psi_u_transformed = psi_u(g);
  const Vector3 lifted_equivariant =
      Lift(psi_u_transformed)(Symmetry::Orbit{eta1}(g));

  EXPECT(assert_equal(lifted_ad, lifted_equivariant));
}

//==============================================================================
// Prediction Tests
//==============================================================================
TEST(EquivariantFilter_Attitude, Predict) {
  using namespace attitude_example;

  // Initial group and reference state: both identity.x
  Matrix2 Sigma0 = 0.01 * I_2x2;
  EquivariantFilter<M, Symmetry> filter(eta_ref, Sigma0);

  // --- Perform prediction through EqF ---
  InputOrbit psi_u(omega);
  Matrix3 Sigma_u = 0.1 * I_3x3;
  Matrix3 Q = processNoise(Sigma_u);
  Matrix23 B = inputMatrixB(Q0);
  Matrix2 Qc = B * Q * B.transpose();  // manifold continuous-time covariance
  const double dt = 0.01;
  filter.predict(lift_omega, psi_u, Qc, dt);

  // --- Expected result ---
  // X_new = X_old * Exp(omega * dt) (Right action predict or left?)
  // Filter impl: X_ = Compose(X_, Exp(Lambda*dt)) -> X_new = X_old * Exp.
  const G X_expected = Q0 * Rot3::Expmap(omega * dt);
  EXPECT(assert_equal(X_expected, filter.groupEstimate()));

  // --- Expected covariance update ---
  Matrix2 Phi = I_2x2;

  // Qc is already on manifold, continuous-time.
  Matrix2 Q_process = Qc * dt;
  Matrix2 P_expected = Phi * Sigma0 * Phi.transpose() + Q_process;
  EXPECT(assert_equal(P_expected, filter.errorCovariance()));

  // state() should be the rotated reference direction on S^2
  const Unit3 state_expected(X_expected.unrotate(eta_ref.point3()));
  EXPECT(assert_equal(state_expected, filter.state()));
}

//==============================================================================
TEST(EquivariantFilter_Attitude, CovarianceRotation) {
  using namespace attitude_example;

  Matrix2 Sigma0 = 0.01 * I_2x2;
  EquivariantFilter<M, Symmetry> filter(eta_ref, Sigma0);

  // Move away from identity so the covariance needs to be rotated.
  const double dt = 0.02;
  Matrix3 Sigma_u = 0.1 * I_3x3;
  Matrix3 Q = processNoise(Sigma_u);
  Matrix23 B = inputMatrixB(Q0);
  Matrix2 Qc = B * Q * B.transpose();
  filter.predict(lift_omega, psi_u, Qc, dt);

  Matrix2 P_error = filter.errorCovariance();
  Matrix2 J;
  const typename Symmetry::Diffeomorphism action_at_g(filter.groupEstimate());
  action_at_g(eta_ref, &J);
  Matrix2 P_expected = J.transpose() * P_error * J;

  EXPECT(assert_equal(P_expected, filter.covariance(), 1e-9));
}

//==============================================================================
TEST(EquivariantFilter_Attitude, Update) {
  using namespace attitude_example;

  // 1. Setup Filter
  Matrix2 Sigma0 = 0.01 * I_2x2;
  EquivariantFilter<M, Symmetry> filter(eta_ref, Sigma0);

  // 2. Predict to move away from identity
  const double dt = 0.01;
  Matrix3 Sigma_u = 0.1 * I_3x3;
  Matrix3 Q = processNoise(Sigma_u);
  Matrix23 B = inputMatrixB(Q0);
  Matrix2 Qc = B * Q * B.transpose();  // manifold continuous-time covariance
  filter.predict(lift_omega, psi_u, Qc, dt);

  const G Q_before = filter.groupEstimate();
  const Matrix2 P_before = filter.errorCovariance();

  // 3. Setup Measurement
  const Vector3 z = c_m * eta_ref.point3();
  const Matrix3 R_meas = 0.01 * I_3x3;
  MeasurementFunctor h(c_m);

  // 4. Run Filter Update
  filter.update(h, z, R_meas);

  const G Q_after = filter.groupEstimate();
  const Matrix2 P_after = filter.errorCovariance();

  // 5. Run Manual Update (Mirroring EquivariantFilter implementation)

  // Re-calculate InnovationLift (PseudoInverse of Dphi at identity)
  Matrix23 Dphi0;
  phi_ref(G::Identity(), Dphi0);
  Matrix32 InnovationLift =
      Dphi0.completeOrthogonalDecomposition().pseudoInverse();

  // Re-calculate Measurement Matrix H
  const M eta_hat = phi_ref(Q_before);
  Matrix H;
  const Vector3 z_hat = h(eta_hat, H);

  // Calculate Gain K
  Matrix S = H * P_before * H.transpose() + R_meas;
  Matrix K = P_before * H.transpose() * S.inverse();

  // Calculate Innovation
  const Vector3 innovation = z_hat - z;

  // Calculate Correction
  Vector2 delta_xi = -K * innovation;
  Vector3 delta_x = InnovationLift * delta_xi;

  // Update State: X_new = Exp(delta_x) * X_old (Left Update)
  const G X_expected = Rot3::Expmap(delta_x) * Q_before;

  // Update Covariance: Joseph Form
  Matrix2 I_KC = Matrix2::Identity() - K * H;
  Matrix2 P_expected =
      I_KC * P_before * I_KC.transpose() + K * R_meas * K.transpose();

  // 6. Assertions
  EXPECT(assert_equal(P_expected, P_after, 1e-9));
  EXPECT(assert_equal(X_expected, Q_after, 1e-9));

  const Unit3 state_expected(Q_after.unrotate(eta_ref.point3()));
  EXPECT(assert_equal(state_expected, filter.state(), 1e-9));
}

//==============================================================================
TEST(EquivariantFilter_Attitude, CheckMatrices) {
  using namespace attitude_example;

  // Initial group and reference state: both identity.
  const G Q0 = Rot3::Ypr(0.1, 0.2, 0.3);
  const M eta_ref(1, 2, 3);
  Matrix2 Sigma0 = 0.01 * I_2x2;

  EquivariantFilter<M, Symmetry> filter(eta_ref, Sigma0);

  // Check A matrix
  InputOrbit psi_u(omega);
  Matrix2 A_computed =
      filter.computeErrorDynamicsMatrix<Lift, InputOrbit>(psi_u);
  Matrix2 A_provided = stateMatrixA(Q0);
  EXPECT(assert_equal(A_provided, A_computed));

  // Check C matrix
  const M eta_hat = phi_ref(Q0);
  Matrix C_computed;
  MeasurementFunctor h(c_m);
  h(eta_hat, C_computed);
  EXPECT(C_computed.rows() == 3 && C_computed.cols() == 2);
}

//==============================================================================

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
