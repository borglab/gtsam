/**
 * @file testABC.cpp
 * @brief Test file for ABC (Attitude-Bias-Calibration) system components
 *
 * @author Darshan Rajasekaran
 * @author Jennifer Oum
 * @author Rohan Bansal
 * @author Frank Dellaert
 * @date 2025
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/navigation/EquivariantFilter.h>
#include <gtsam/navigation/LieGroupEKF.h>
#include <gtsam_unstable/geometry/ABC.h>

using namespace gtsam;

// Define N for testing purposes, e.g., 2 calibration states
using State = abc::State<2>;
using Group = abc::Group<2>;
using Symmetry = abc::Symmetry<2>;
using Lift = abc::Lift<2>;
using InputOrbit = abc::InputAction<2>::Orbit;
using OutputOrbit = abc::OutputAction<2>::Orbit;
using Innovation = abc::Innovation<2>;
using Calibrations = abc::Calibrations<2>;

/* ************************************************************************* */
namespace abc_examples {
const Rot3 A1 = Rot3::Rx(0.1);
const Vector3 t1(0.01, 0.02, 0.03);
const Calibrations B1{Rot3::Ry(0.05), Rot3::Rz(0.06)};
const State xi1(A1, t1, B1);
const Group g1{{A1, t1}, B1};

const Rot3 A2 = Rot3::Ry(0.2);
const Vector3 t2(0.04, 0.05, 0.06);
const Calibrations B2{Rot3::Rz(0.07), Rot3::Rx(0.08)};
const State xi2(A2, t2, B2);
const Group g2{{A2, t2}, B2};

const Vector3 omega(1, 2, 3);
const Vector6 u = abc::toInputVector(omega);

const Vector3 omega2(0.01, -0.02, 0.015);
const Vector6 u2 = abc::toInputVector(omega2);

const Unit3 y(1, 0, 0), d(0, 1, 0);
}  // namespace abc_examples

/* ************************************************************************* */
// State manifold construction, retract, and local coordinates.
TEST(ABC, State) {
  State state1 = abc_examples::xi1;

  EXPECT(assert_equal(abc_examples::A1, state1.R));
  EXPECT(assert_equal(abc_examples::t1, state1.b));
  EXPECT(assert_equal(abc_examples::B1, state1.S));

  // Test identity
  State identityState = State::identity();
  EXPECT(assert_equal(identityState.R, Rot3()));
  EXPECT(assert_equal<Vector3>(identityState.b, Z_3x1));
  Calibrations expectedS_id;
  EXPECT(assert_equal(identityState.S, expectedS_id));

  // Test localCoordinates and retract (manifold properties)
  Rot3 R2 = Rot3::Rx(0.2);
  Vector3 b2(0.05, 0.06, 0.07);
  Calibrations S2;
  S2[0] = Rot3::Ry(0.1);
  S2[1] = Rot3::Rz(0.15);
  State state2(R2, b2, S2);

  Vector actual_local = state1.localCoordinates(state2);
  State retracted_state2 = state1.retract(actual_local);
  EXPECT(assert_equal(retracted_state2.R, state2.R));
  EXPECT(assert_equal(retracted_state2.b, state2.b));
  EXPECT(assert_equal(retracted_state2.S, state2.S));

  // Test localCoordinates at identity
  Vector expected_identity_local = Vector::Zero(6 + 3 * 2);
  EXPECT(assert_equal(identityState.localCoordinates(identityState),
                      expected_identity_local));

  // Test retract at identity
  Vector v_test = Vector::Zero(6 + 3 * 2);
  v_test.head<3>() << 0.1, 0.2, 0.3;         // R
  v_test.segment<3>(3) << 0.01, 0.02, 0.03;  // b
  v_test.segment<3>(6) << 0.05, 0.06, 0.07;  // S[0]
  v_test.segment<3>(9) << 0.08, 0.09, 0.10;  // S[1]

  State retracted_from_id = identityState.retract(v_test);
  EXPECT(assert_equal(retracted_from_id.R, Rot3::Expmap(v_test.head<3>())));
  EXPECT(assert_equal(retracted_from_id.b, v_test.segment<3>(3).eval()));
  EXPECT(assert_equal(retracted_from_id.S[0],
                      Rot3::Expmap(v_test.segment<3>(6).eval())));
  EXPECT(assert_equal(retracted_from_id.S[1],
                      Rot3::Expmap(v_test.segment<3>(9).eval())));
}

/* ************************************************************************* */
// Group operations (compose, inverse, retract) behave as expected.
TEST(ABC, GroupOperations) {
  using namespace abc_examples;

  // Test group multiplication
  Group g1_g2 = g1 * g2;
  {
    auto [A, a, B] = abc::asTriple<2>(g1_g2);
    EXPECT(assert_equal(A1 * A2, A));
    Vector3 expected_a = t1 + A1.matrix() * t2;
    EXPECT(assert_equal(expected_a, a));
    EXPECT(assert_equal(B1[0] * B2[0], B[0]));
    EXPECT(assert_equal(B1[1] * B2[1], B[1]));
  }

  // Test inverse
  Group g1_inv = g1.inverse();
  {
    auto [A, a, B] = abc::asTriple<2>(g1_inv);
    EXPECT(assert_equal(A1.inverse(), A));
    Vector3 expected_a_inv = -A1.inverse().matrix() * t1;
    EXPECT(assert_equal(expected_a_inv, a));
    EXPECT(assert_equal(B1[0].inverse(), B[0]));
    EXPECT(assert_equal(B1[1].inverse(), B[1]));
  }

  // Test g * g.inv() == identity
  Group identity_check = g1 * g1_inv;
  {
    auto [A, a, B] = abc::asTriple<2>(identity_check);
    Group expected_identity = Group::Identity();
    EXPECT(assert_equal(Rot3(), A));
    EXPECT(assert_equal(Vector3(0, 0, 0), a));
    EXPECT(assert_equal(Calibrations{Rot3(), Rot3()}, B));
  }

  // Test Expmap and Logmap
  Group::TangentVector v_tangent = Group::TangentVector::Zero();
  v_tangent.head<3>() << 0.1, 0.2, 0.3;         // For A
  v_tangent.segment<3>(3) << 0.01, 0.02, 0.03;  // For 'a' part
  v_tangent.segment<3>(6) << 0.04, 0.05, 0.06;  // For B[0]
  v_tangent.segment<3>(9) << 0.07, 0.08, 0.09;  // For B[1]

  Group g_exp = Group::Expmap(v_tangent);
  EXPECT(assert_equal(v_tangent, Group::Logmap(g_exp)));

  // Test retract on G
  Group g_retracted = g1.expmap(v_tangent);
  const Group composed = g1 * Group::Expmap(v_tangent);
  {
    auto [rA, ra, rB] = abc::asTriple<2>(g_retracted);
    auto [cA, ca, cB] = abc::asTriple<2>(composed);
    EXPECT(assert_equal(cA, rA));
    EXPECT(assert_equal(ca, ra));
    EXPECT(assert_equal(cB, rB));
  }

  // Test traits for G
  {
    const Group identity = traits<Group>::Identity();
    auto [A, a, B] = abc::asTriple<2>(identity);
    EXPECT(assert_equal(Rot3(), A));
    EXPECT(assert_equal(Vector3(0, 0, 0), a));
    EXPECT(assert_equal(Calibrations{Rot3(), Rot3()}, B));
  }
}

/* ************************************************************************* */
/*
 * Matrix representation of the Lie-algebra adjoint operator ad_xi on g. For
 * this direct product group it is block-diagonal with Pose3 and Rot3 blocks.
 */
static Group::Jacobian adjointMap(const Group::TangentVector& xi) {
  Group::Jacobian result = Group::Jacobian::Zero();
  result.block<6, 6>(0, 0) = Pose3::adjointMap(xi.head<6>());
  for (size_t i = 0; i < 2; ++i) {
    result.block<3, 3>(6 + 3 * i, 6 + 3 * i) =
        Rot3::adjointMap(xi.segment<3>(6 + 3 * i));
  }
  return result;
}

//******************************************************************************
// Adjoint and adjointMap should match block-diagonal expectations.
TEST(ABC, AdjointMap) {
  using namespace abc_examples;

  Group::Jacobian Ad1 = g1.AdjointMap();
  Group::Jacobian expected = Group::Jacobian::Zero();
  expected.block<6, 6>(0, 0) = Pose3(A1, t1).AdjointMap();
  for (size_t i = 0; i < 2; ++i) {
    expected.block<3, 3>(6 + 3 * i, 6 + 3 * i) = B1[i].AdjointMap();
  }

  EXPECT(assert_equal(Ad1, expected));

  Group::TangentVector xi = Group::TangentVector::Zero();
  xi.head<3>() << 0.1, -0.2, 0.3;
  xi.segment<3>(3) << 0.01, 0.02, 0.03;
  xi.segment<3>(6) << 0.05, -0.04, 0.02;
  xi.segment<3>(9) << -0.03, 0.07, -0.01;

  Group::Jacobian ad_xi = adjointMap(xi);
  Group::Jacobian expected_ad = Group::Jacobian::Zero();
  expected_ad.block<6, 6>(0, 0) = Pose3::adjointMap(xi.head<6>());
  expected_ad.block<3, 3>(6, 6) = Rot3::adjointMap(xi.segment<3>(6));
  expected_ad.block<3, 3>(9, 9) = Rot3::adjointMap(xi.segment<3>(9));

  EXPECT(assert_equal(ad_xi, expected_ad));
}

/* ************************************************************************* */
// Basic sanity check on state action results.
TEST(ABC, Symmetry) {
  using namespace abc_examples;

  // Basic sanity check on state action
  Symmetry phi;
  State transformed_xi = phi(xi2, g1);
  EXPECT(assert_equal(transformed_xi.R, xi2.R * A1));
  auto invA1 = A1.inverse();
  EXPECT(assert_equal<Matrix>(transformed_xi.b, invA1.rotate(xi2.b - t1)));
  EXPECT(assert_equal(transformed_xi.S[0], invA1 * xi2.S[0] * B1[0], 1e-9));
  EXPECT(assert_equal(transformed_xi.S[1], invA1 * xi2.S[1] * B1[1], 1e-9));
}

/* ************************************************************************* */
// Check analytic Jacobians of the symmetry action against numerical
// derivatives.
TEST(ABC, SymmetryJacobians) {
  using namespace abc_examples;

  Symmetry phi;

  Matrix Hxi, Hg;
  phi(xi1, g2, Hxi, Hg);

  std::function<State(const State&)> action_wrt_state = [&](const State& xi) {
    return phi(xi, g2);
  };
  std::function<State(const Group&)> action_wrt_group = [&](const Group& g) {
    return phi(xi1, g);
  };

  Matrix Hxi_numeric = numericalDerivative11(action_wrt_state, xi1, 1e-7);
  Matrix Hg_numeric = numericalDerivative11(action_wrt_group, g2, 1e-7);

  EXPECT(assert_equal(Hxi_numeric, Hxi, 1e-6));
  EXPECT(assert_equal(Hg_numeric, Hg, 1e-6));
}

/* ************************************************************************* */
// Verify symmetry defines a right action.
TEST(ABC, SymmetryIsRightAction) {
  using namespace abc_examples;

  EXPECT_RIGHT_ACTION(Symmetry(), xi1, g1, g2);
  EXPECT_RIGHT_ACTION(Symmetry(), xi1, g2, g1);
}

/* ************************************************************************* */
// Lift functor should match analytical expression.
TEST(ABC, LiftFunctor) {
  State xi = abc_examples::xi1;

  // Setup input
  Vector3 omega(0.5, 0.6, 0.7);
  Vector6 u = abc::toInputVector(omega);
  typename Group::TangentVector L = Lift(u)(xi);

  // Expected values
  Vector3 expected_L_head = omega - xi.b;
  Vector3 expected_L_segment3 = -Rot3::Hat(omega) * xi.b;
  Vector3 expected_L_segment6_0 = xi.S[0].inverse().matrix() * expected_L_head;
  Vector3 expected_L_segment6_1 = xi.S[1].inverse().matrix() * expected_L_head;

  EXPECT(assert_equal<Vector>(L.head<3>(), expected_L_head));
  EXPECT(assert_equal<Vector>(L.segment<3>(3), expected_L_segment3));
  EXPECT(assert_equal<Vector>(L.segment<3>(6), expected_L_segment6_0));
  EXPECT(assert_equal<Vector>(L.segment<3>(9), expected_L_segment6_1));
}

/* ************************************************************************* */
namespace abc_input_action_example {
Group X_hat = abc_examples::g1;
Vector6 u = abc::toInputVector(abc_examples::omega);
InputOrbit psi_u(u);
Symmetry::Orbit phi_xi1(abc_examples::xi1);
State state_est = phi_xi1(X_hat);
Lift lift_u(u);
Group::TangentVector xi = lift_u(state_est);
}  // namespace abc_input_action_example

/* ************************************************************************* */
// Input action should map angular velocity as defined.
TEST(ABC, InputOrbit) {
  using abc_examples::A1;
  using abc_examples::t1;
  using namespace abc_input_action_example;

  Vector6 transformed_u = psi_u(X_hat);
  EXPECT(assert_equal<Vector>(transformed_u.head<3>(),
                              A1.unrotate(abc_examples::omega - t1)));
  EXPECT(assert_equal<Vector>(transformed_u.tail<3>(), Z_3x1,
                              1e-9));  // Virtual input stays zero

  EXPECT(assert_equal(transformed_u, InputOrbit(u)(X_hat)));
}

/* ************************************************************************* */
// Input action must satisfy right-action property.
TEST(ABC, InputActionIsRightAction) {
  using namespace abc_examples;

  EXPECT_RIGHT_ACTION(abc::InputAction<2>(), u, g1, g2);
  EXPECT_RIGHT_ACTION(abc::InputAction<2>(), u, g2, g1);
}

/* ************************************************************************* */
// Manifold dynamics ξ̇ = f(ξ, ω) should be equivariant under the state action.
TEST(ABC, ManifoldDynamicsEquivariance) {
  using namespace abc_examples;

  const auto f_u = [&](const State& xi) -> Vector {
    return abc::dynamics<2>(omega, xi);
  };
  using InducedField = Symmetry::InducedVectorField<decltype(f_u)>;
  const InducedField fInduced(g1, f_u);

  Matrix H;
  const State transported = Symmetry::Orbit{xi1}(g1.inverse());
  Symmetry::Diffeomorphism{g1}(transported, H);
  const Vector expected = H * f_u(transported);
  const Vector induced = fInduced(xi1);

  EXPECT(assert_equal(expected, induced));

  // Verify equivariance property: f(φ(xi, g), psi_u(g)) = φ_*(f(xi, u), g)
  // Equation (3.5) in Fornasier thesis
  InputOrbit psi_u(u);
  const Vector3 omega_transformed = psi_u(g1).head<3>();
  const auto equivariant =
      abc::dynamics<2>(omega_transformed,
                       xi1);  // dynamics given input-transformed omega
  EXPECT(assert_equal(induced, equivariant));
}

/* ************************************************************************* */
// Lift Jacobian w.r.t. state should match numerical derivative.
TEST(ABC, LiftJacobians) {
  using namespace abc_examples;

  Lift lift_u(u);

  Matrix H;
  const Group::TangentVector lifted = lift_u(xi1, H);

  std::function<Group::TangentVector(const State&)> lift_wrt_state =
      [&](const State& xi) { return lift_u(xi); };

  const Matrix H_numeric = numericalDerivative11(lift_wrt_state, xi1, 1e-7);

  EXPECT(assert_equal(lift_u(xi1), lifted));
  EXPECT(assert_equal(H_numeric, H, 1e-6));
}

/* ************************************************************************* */
// Lift should yield original dynamics on the manifold.
TEST(ABC, LiftShadowManifoldDynamics) {
  using namespace abc_examples;

  Symmetry::Orbit phi_xi(xi1);
  Matrix H;
  phi_xi(Group::Identity(), H);  // derivative w.r.t. group at identity

  const Group::TangentVector lifted = Lift(u)(xi1);
  const Vector shadow = H * lifted;

  const Vector manifold = abc::dynamics<2>(omega, xi1);
  EXPECT(assert_equal(shadow, manifold));
}

/* ************************************************************************* */
// Lift should be equivariant under the state and input actions.
TEST(ABC, LiftEquivariance) {
  using namespace abc_examples;

  // Check Ad_{X^{-1}} Λ(ξ, u) = Λ(φ_X(ξ), ψ_X(u)).
  Lift lift_omega(u);
  InputOrbit psi_u(u);

  const Group X = g1;

  const Group::TangentVector lifted = lift_omega(xi1);
  const Group::TangentVector lifted_ad = X.inverse().AdjointMap() * lifted;

  const Group::TangentVector lifted_equivariant =
      Lift(psi_u(X))(Symmetry::Orbit{xi1}(X));

  EXPECT(assert_equal(lifted_ad, lifted_equivariant));
}

/* ************************************************************************* */
// Validate legacy state-transition matrix computation.
TEST(ABC, InputAction_stateMatrixA) {
  using namespace abc_input_action_example;

  Matrix A_matrix = abc::stateMatrixA(psi_u, X_hat);
  Matrix3 W0 = Rot3::Hat(psi_u(X_hat.inverse()).head<3>());

  Matrix expected_A1 = Matrix::Zero(6, 6);
  expected_A1.block<3, 3>(0, 3) = -I_3x3;
  expected_A1.block<3, 3>(3, 3) = W0;

  Matrix expected_A2 = gtsam::diag({W0, W0});
  Matrix expected_A_matrix = gtsam::diag({expected_A1, expected_A2});

  EXPECT(assert_equal(expected_A_matrix, A_matrix));
}

/* ************************************************************************* */
// Compare EqF-computed A/C with legacy helpers.
// These tests succeed when xi_ref is State::identity().
TEST(ABC, ComputeErrorDynamicsMatrixMatchesLegacy) {
  using namespace abc_examples;

  const State xi_ref = State::identity();
  Matrix initialSigma = Matrix::Identity(12, 12);
  const auto& X_hat = g2;
  EquivariantFilter<State, abc::Symmetry<2>> filter(xi_ref, initialSigma,
                                                    X_hat);

  auto phi_ref = Symmetry::Orbit{xi_ref};
  const State xi_hat = phi_ref(X_hat);

  // A_provided should now be the Manifold-space matrix
  // stateMatrixA returns the correct Manifold-space matrix (DimM x DimM)
  InputOrbit psi_u(u);
  Matrix expected_A = abc::stateMatrixA(psi_u, X_hat);

  // A_computed is now computed on Manifold (D_act * D_lift)
  Matrix A_computed = filter.computeErrorDynamicsMatrix<Lift>(psi_u);

  EXPECT(assert_equal(expected_A, A_computed, 1e-9));

  // Calculate A as in VanGoor23thesis, Formula 5.23
  // The chart for xi_ref==identity is just identity matrices
  auto u0 = psi_u(X_hat.inverse());  // origin velocity

  Matrix D_act;
  phi_ref(Group::Identity(), &D_act);

  Matrix D_lift;
  Lift lift_u0(u0);
  lift_u0(xi_ref, &D_lift);

  Matrix A_decomposed = D_act * D_lift;
  EXPECT(assert_equal(expected_A, A_decomposed, 1e-9));
}

/* ************************************************************************* */
// dynamics Derivative in filter should match numerical derivative.
TEST(ABC, ComputeErrorDynamicsMatrix) {
  using namespace abc_examples;

  // Reference error state and current group estimate
  const State xi_ref = State::identity();
  const Group& X_hat = g2;

  // Origin velocity u0 in the body/origin frame
  InputOrbit psi_u(u);
  const Vector6 u0 = psi_u(X_hat.inverse());
  Lift lift_u0(u0);

  // Ensure the reference is an equilibrium for the error dynamics by
  // subtracting a constant lift term.
  const Group::TangentVector lxi_ref = lift_u0(xi_ref);

  const double dt = 1e-4;

  // One-step flow map for the error state e under explicit Euler integration:
  //   e(dt) ≈ e ⊕ (F(e) * dt),
  // where F(e) = Dφ_e ( Λ(e, u0) - Λ(xi_ref, u0) ).
  std::function<State(const State&)> Phi_dt = [&](const State& e0) {
    // Dφ_e evaluated at the identity of the group
    Symmetry::Orbit phi_e(e0);
    Matrix Dphi_e;
    phi_e(Group::Identity(), Dphi_e);

    // Lifted dynamics at e and at the reference
    const Group::TangentVector le = lift_u0(e0);
    const Vector fe = Dphi_e * (le - lxi_ref);

    // Explicit Euler step on the manifold
    return e0.retract(fe * dt);
  };

  // Numerically approximate the Jacobian of the flow at the reference error
  // state and recover A from DΦ_dt ≈ I + dt * A.
  const Matrix Phi_numeric = numericalDerivative11(Phi_dt, xi_ref, 1e-7);
  const Matrix I = Matrix::Identity(Phi_numeric.rows(), Phi_numeric.cols());
  const Matrix A_numeric = (Phi_numeric - I) / dt;

  // Analytic error-dynamics matrix from the EquivariantFilter implementation
  Matrix initialSigma = Matrix::Identity(12, 12);
  EquivariantFilter<State, abc::Symmetry<2>> filter(xi_ref, initialSigma,
                                                    X_hat);
  Matrix A_computed = filter.computeErrorDynamicsMatrix<Lift>(psi_u);

  EXPECT(assert_equal(A_numeric, A_computed, 1e-7));
}

/* ************************************************************************* */
// State transition matrix for ABC system under InputOrbit dynamics
// This is the old code for stateTransitionMatrix, kept so we can keep testing
// against it, specifically that it matches the LieGroupEKF transition matrix.
template <size_t N, typename InputOrbit>
static Matrix stateTransitionMatrix(const InputOrbit& psi_u, const Group& X_hat,
                                    double dt) {
  const Vector3 omega_tilde = psi_u(X_hat.inverse()).template head<3>();
  Matrix3 W0 = Rot3::Hat(omega_tilde);
  Matrix Phi1 = Matrix::Zero(6, 6);
  Matrix3 W0_sq = W0 * W0;
  Matrix3 Phi12 = -dt * (I_3x3 + 0.5 * dt * W0 + (dt * dt / 6.0) * W0_sq);
  Matrix3 Phi22 = I_3x3 + dt * W0 + 0.5 * dt * dt * W0_sq;
  Phi1.block<3, 3>(0, 0) = I_3x3;
  Phi1.block<3, 3>(0, 3) = Phi12;
  Phi1.block<3, 3>(3, 3) = Phi22;

  std::vector<Matrix> blocks;
  blocks.push_back(Phi1);
  blocks.insert(blocks.end(), N, Phi22);
  return gtsam::diag(blocks);
}

/* ************************************************************************* */
// Closed-form transition should match expected Phi.
TEST(ABC, InputAction_stateTransitionMatrix) {
  using namespace abc_input_action_example;

  // Setup input
  Vector3 omega(0.5, 0.6, 0.7);
  double dt = 0.1;

  Vector6 u = abc::toInputVector(omega);
  InputOrbit psi_u(u);
  Matrix Phi = stateTransitionMatrix<2>(psi_u, X_hat, dt);
  Matrix3 W0 = Rot3::Hat(psi_u(X_hat.inverse()).head<3>());
  Matrix Phi1 = Matrix::Zero(6, 6);
  Matrix3 Phi12 = -dt * (I_3x3 + (dt / 2) * W0 + ((dt * dt) / 6) * W0 * W0);
  Matrix3 Phi22 = I_3x3 + dt * W0 + ((dt * dt) / 2) * W0 * W0;

  Phi1.block<3, 3>(0, 0) = I_3x3;
  Phi1.block<3, 3>(0, 3) = Phi12;
  Phi1.block<3, 3>(3, 3) = Phi22;
  Matrix Phi2 = gtsam::diag({Phi22, Phi22});
  Matrix expected_Phi = gtsam::diag({Phi1, Phi2});

  EXPECT(assert_equal(expected_Phi, Phi));
}

/* ************************************************************************* */
// Transition should match LieGroupEKF reference (K=2).
TEST(ABC, InputAction_stateTransitionMatchesLieGroupEKF) {
  using namespace abc_input_action_example;

  double dt = 1e-4;

  Matrix Phi_expected = stateTransitionMatrix<2>(psi_u, X_hat, dt);

  Group::Jacobian ad_xi = adjointMap(xi);
  Group::Jacobian Df = abc::stateMatrixA(psi_u, X_hat) + ad_xi;
  Group::Jacobian P0 = Group::Jacobian::Identity();
  LieGroupEKF<Group> ekf(X_hat, P0);

  Group::Jacobian Dexp;
  Group U = Group::Expmap(xi * dt, &Dexp);

  // Below does the same as
  // Group::Jacobian Phi_ekf = ekf.transitionMatrix<2>(xi, Df, dt, U, Dexp);
  // We inline as Group does not have adjointMap member function.
  const size_t K = 2;
  const Matrix A = Df - ad_xi;
  Group::Jacobian Phi_ekf = expm(A * dt, K);

  EXPECT(assert_equal(Phi_expected, Phi_ekf, 2e-5));
}

/* ************************************************************************* */
// Transition should match LieGroupEKF reference (K=1).
TEST(ABC, InputAction_stateTransitionMatchesLieGroupEKF_K1) {
  using namespace abc_input_action_example;

  double dt = 1e-4;

  Group::Jacobian Df = abc::stateMatrixA(psi_u, X_hat) + adjointMap(xi);
  Group::Jacobian P0 = Group::Jacobian::Identity();
  LieGroupEKF<Group> ekf(X_hat, P0);

  Group::Jacobian Dexp;
  Group U = Group::Expmap(xi * dt, &Dexp);

  Group::Jacobian Phi_ekf = ekf.transitionMatrix<1>(xi, Df, dt, U, Dexp);

  Group::Jacobian Phi_expected = stateTransitionMatrix<2>(psi_u, X_hat, dt);

  EXPECT(assert_equal(Phi_expected, Phi_ekf, 1e-6));
}

/* ************************************************************************* */
// Input matrix Bt should match block-diagonal structure.
TEST(ABC, InputAction_inputMatrix) {
  using namespace abc_input_action_example;
  using abc_examples::A1;
  using abc_examples::B1;

  Matrix input_matrix = abc::inputMatrixB(X_hat);

  const Matrix3 A = A1.matrix();
  Matrix expected_B1 = gtsam::diag({A, A});
  Matrix expected_B2(3 * 2, 3 * 2);
  expected_B2.setZero();
  for (size_t i = 0; i < 2; ++i) {
    expected_B2.block<3, 3>(3 * i, 3 * i) = B1[i].matrix();
  }
  Matrix expected_input_matrix = gtsam::diag({expected_B1, expected_B2});

  EXPECT(assert_equal(input_matrix, expected_input_matrix));
}

/* ************************************************************************* */
// Process noise embedding should match expected block diagonal.
TEST(ABC, InputAction_processNoise) {
  Matrix Sigma6 =
      (Matrix(6, 6) << 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0,
       0, 0, 4, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 6)
          .finished();

  Matrix Q = abc::inputProcessNoise<2>(Sigma6);

  Matrix expected_Q = gtsam::diag({Sigma6, 1e-9 * I_3x3, 1e-9 * I_3x3});  // N=2

  EXPECT(assert_equal(Q, expected_Q));
}

/* ************************************************************************* */
// Output action should handle calibrated/uncalibrated indices.
TEST(ABC, OutputOrbit) {
  using namespace abc_examples;

  // Test outputAction (calibrated sensor)
  int cal_idx = 0;
  OutputOrbit phi_y(y.unitVector(), abc::OutputAction<2>(cal_idx));
  Vector3 transformed_y_calibrated = phi_y(g1);
  EXPECT(assert_equal<Vector>(transformed_y_calibrated,
                              B1[0].unrotate(y.unitVector())));

  // Test outputAction (uncalibrated sensor)
  int uncalibrated_idx = -1;
  OutputOrbit uncalibrated_phi_y(y.unitVector(),
                                 abc::OutputAction<2>(uncalibrated_idx));
  Vector3 transformed_y_uncalibrated = uncalibrated_phi_y(g1);
  EXPECT(assert_equal<Vector>(transformed_y_uncalibrated,
                              A1.unrotate(y.unitVector())));
}

/* ************************************************************************* */
// Output action must satisfy right-action property.
TEST(ABC, OutputActionIsRightAction) {
  using namespace abc_examples;

  EXPECT_RIGHT_ACTION(abc::OutputAction<2>(0), y.unitVector(), g1, g2);
  EXPECT_RIGHT_ACTION(abc::OutputAction<2>(0), y.unitVector(), g2, g1);
}

/* ************************************************************************* */
// Measurement matrix C should match legacy computation.
TEST(ABC, OutputAction_measurementMatrixC) {
  using namespace abc_examples;
  Matrix3 wedge_d = Rot3::Hat(d.unitVector());

  // Test with calibrated sensor (idx = 0)
  int cal_idx = 0;
  Matrix C_cal = abc::measurementMatrixC<2>(d, cal_idx);

  Matrix expected_Cc_cal = Matrix::Zero(3, 3 * 2);
  expected_Cc_cal.block<3, 3>(0, 3 * cal_idx) = wedge_d;

  Matrix expected_temp_cal(3, 6 + 3 * 2);
  expected_temp_cal.block<3, 3>(0, 0) = wedge_d;
  expected_temp_cal.block<3, 3>(0, 3) = Matrix3::Zero();
  expected_temp_cal.block(0, 6, 3, 3 * 2) = expected_Cc_cal;
  Matrix expected_C_cal = wedge_d * expected_temp_cal;

  EXPECT(assert_equal(C_cal, expected_C_cal));
}

//==============================================================================
// Compare EqF-computed A/C with legacy helpers.
TEST(ABC, ComputeMeasurementMatrix) {
  using namespace abc_examples;

  const Group g_0;
  const State xi_ref = xi1;
  Matrix initialSigma = Matrix::Identity(12, 12);
  EquivariantFilter<State, abc::Symmetry<2>> filter(xi_ref, initialSigma);

  // Check C matrix
  Innovation innovation(y, d, 0, xi_ref);
  Matrix C_computed;
  (void)innovation(xi_ref, C_computed);  // to compute any internal values
  Matrix C_legacy = abc::measurementMatrixC<2>(d, 0);
  EXPECT(assert_equal(C_legacy, C_computed, 1e-9));
}

/* ************************************************************************* */
// Regression expectations for EqFilter predict/update.
namespace abc_eqf_regression {
const Group expected_predict({Rot3(1, 0.00015, -0.0004,  //
                                   -0.00015, 1, 3e-08,   //
                                   0.0004, 3e-08, 1),
                              Point3(9.00091e-06, 1.49932e-06, -3.9982e-06)},
                             {Rot3(1, 0.000149811, -0.000400001,   //
                                   -0.000149814, 1, -7.46691e-06,  //
                                   0.000399999, 7.52684e-06, 1),
                              Rot3(1, 0.000150005, -0.000399278,  //
                                   -0.000149995, 1, 2.40155e-05,  //
                                   0.000399282, -2.39557e-05, 1)});

const Matrix expected_P_after_predict =
    (Matrix(12, 12) << 0.110001, -0, 0, -0.0001, 0, -0, 0, 0, 0, 0, 0, 0,  //
     -0, 0.110001, -0, 0, -0.0001, -0, 0, 0, 0, 0, 0, 0,                   //
     0, -0, 0.110001, 0, 0, -0.0001, 0, 0, 0, 0, 0, 0,                     //
     -0.0001, 0, 0, 0.02, 0, -0, 0, 0, 0, 0, 0, 0,                         //
     -0, -0.0001, 0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0,                         //
     -0, -0, -0.0001, -0, 0, 0.02, 0, 0, 0, 0, 0, 0,                       //
     0, 0, 0, 0, 0, 0, 1, 0, -0, 0, 0, 0,                                  //
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,                                   //
     0, 0, 0, 0, 0, 0, -0, 0, 1, 0, 0, 0,                                  //
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, -0,                                //
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0,                                 //
     0, 0, 0, 0, 0, 0, 0, 0, 0, -0, 0, 0.1)
        .finished();

const Group expected_after_update({Rot3(0.995195, -0.097908, -0.000400003,  //
                                        0.097908, 0.995195, 2.98008e-08,    //
                                        0.000398078, -3.91931e-05, 1),
                                   Point3(0.00201816, -0.000882995,
                                          8.60911e-05)},
                                  {Rot3(0.548024, -0.836459, -0.00263326,  //
                                        0.83646, 0.548012, 0.00413976,     //
                                        -0.00201968, -0.0044713, 0.999988),
                                   Rot3(0.995195, -0.097908, -0.000399281,  //
                                        0.097908, 0.995195, 2.40155e-05,    //
                                        0.000395012, -6.2993e-05, 1)});

const Matrix expected_P_after_update =
    (Matrix(12, 12) <<  //
         0.0991972,
     -0, 0, -9.01785e-05, 0, -0, -0.0982151, 0, 0, 0, 0, 0,       //
     -0, 0.110001, -0, 0, -0.0001, -0, 0, 0, 0, 0, 0, 0,          //
     0, -0, 0.0991972, 0, 0, -0.0001, 0, 0, -0.0982151, 0, 0, 0,  //
     -0.0001, 0, 0, 0.02, 0, -0, 0, 0, 0, 0, 0, 0,                //
     -0, -0.0001, 0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0,                //
     -0, -0, -0.0001, -0, 0, 0.02, 0, 0, 0, 0, 0, 0,              //
     -0.0982151, 0, 0, 0, 0, 0, 0.107144, 0, -0, 0, 0, 0,         //
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,                          //
     0, 0, -0.0982151, 0, 0, 0, -0, 0, 0.107144, 0, 0, 0,         //
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, -0,                       //
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0,                        //
     0, 0, 0, 0, 0, 0, 0, 0, 0, -0, 0, 0.1)
        .finished();
}  // namespace abc_eqf_regression

/* ************************************************************************* */
// Equivariant filter regression for predict/update.
TEST(ABC, EqFilter) {
  using namespace abc_examples;
  using namespace abc_eqf_regression;

  using G = Group;
  const State xi_ref = xi1;  // Reference state (xi circle)

  Matrix initialSigma = Matrix::Identity(G::dimension, G::dimension);
  initialSigma.diagonal().head<3>() =
      Vector3::Constant(0.1);  // Attitude uncertainty
  initialSigma.diagonal().segment<3>(3) =
      Vector3::Constant(0.01);  // Bias uncertainty
  initialSigma.diagonal().tail<3>() =
      Vector3::Constant(0.1);  // Calibration uncertainty

  const G g_0;
  EquivariantFilter<State, Symmetry> filter(xi_ref, initialSigma);

  // Check initial state
  // Check initial state
  EXPECT(assert_equal(g_0, filter.groupEstimate()));

  // Perform a prediction step
  Matrix Sigma = I_6x6;
  double dt = 0.01;
  Matrix Q = abc::inputProcessNoise<2>(Sigma);
  Matrix B = abc::inputMatrixB(g_0);
  Matrix Qc = B * Q * B.transpose();  // manifold continuous-time covariance
  Lift lift_u(u2);
  InputOrbit psi_u(u2);
  filter.predict(lift_u, psi_u, Qc, dt);

  // Regression
  EXPECT(assert_equal(expected_predict, filter.groupEstimate(), 1e-4));
  EXPECT(
      assert_equal(expected_P_after_predict, filter.errorCovariance(), 1e-4));

  // Perform an update step
  const int cal_idx = 0;
  const Matrix3 R = 0.01 * I_3x3;
  Innovation innovation(y, d, cal_idx, xi_ref);
  filter.update<Vector3>(innovation, Z_3x1, R);

  // Regression
  EXPECT(assert_equal(expected_after_update, filter.groupEstimate(), 1e-4));
  EXPECT(assert_equal(expected_P_after_update, filter.errorCovariance(), 1e-4));
}

/* ************************************************************************* */
// Same regression, but using bespoke A/B helpers instead of automatic A.
TEST(ABC, EqFilter_BespokeDynamics) {
  using namespace abc_examples;
  using namespace abc_eqf_regression;

  using G = Group;
  const State xi_ref = xi1;  // Reference state (xi circle)

  Matrix initialSigma = Matrix::Identity(G::dimension, G::dimension);
  initialSigma.diagonal().head<3>() =
      Vector3::Constant(0.1);  // Attitude uncertainty
  initialSigma.diagonal().segment<3>(3) =
      Vector3::Constant(0.01);  // Bias uncertainty
  initialSigma.diagonal().tail<3>() =
      Vector3::Constant(0.1);  // Calibration uncertainty

  const G g_0;
  EquivariantFilter<State, Symmetry> filter(xi_ref, initialSigma);

  EXPECT(assert_equal(g_0, filter.groupEstimate()));

  // Explicit predict with provided A, B, Qc
  Matrix Sigma = I_6x6;
  double dt = 0.01;
  Matrix Q = abc::inputProcessNoise<2>(Sigma);
  Matrix B = abc::inputMatrixB(g_0);
  Matrix Qc = B * Q * B.transpose();
  Lift lift_u(u2);
  InputOrbit psi_u(u2);

  Matrix A = abc::stateMatrixA(psi_u, g_0);
  filter.predictWithJacobian<2>(lift_u, A, Qc, dt);

  EXPECT(assert_equal(expected_predict, filter.groupEstimate(), 1e-4));
  EXPECT(
      assert_equal(expected_P_after_predict, filter.errorCovariance(), 1e-4));

  // Update
  const int cal_idx = 0;
  const Matrix3 R = 0.01 * I_3x3;
  Innovation innovation(y, d, cal_idx, xi_ref);
  filter.update<Vector3>(innovation, Z_3x1, R);

  EXPECT(assert_equal(expected_after_update, filter.groupEstimate(), 1e-4));
  EXPECT(assert_equal(expected_P_after_update, filter.errorCovariance(), 1e-4));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
