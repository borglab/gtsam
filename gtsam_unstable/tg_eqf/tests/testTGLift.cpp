/**
 * @file  testTGLift.cpp
 * @brief The equivariant lift Lambda and the input action psi.
 *
 * Layer above testTGSymmetry.cpp: the system dynamics carried onto the group.
 * The two claims that matter are the closed form against the literal matrix
 * construction, and the lift condition that makes the filter equivariant.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/tg_eqf/TGEqF.h>

using namespace gtsam::tgeqf;
using namespace gtsam;

/* ************************************************************************* */
namespace fixture {

constexpr double kTol = 1e-9;
constexpr double kTolL = 1e-7;

bool veq(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
         double tol = kTol) {
  return assert_equal((Vector)a, (Vector)b, tol);
}

State makeXi() {
  Eigen::Matrix<double, 3, 2> vp;
  vp.col(0) = Vector3(1.0, -2.0, 0.5);
  vp.col(1) = Vector3(0.3, 0.1, -0.4);
  State xi;
  xi.T = ExtendedPose3<2>(Rot3::Rz(0.3) * Rot3::Rx(0.1), vp);
  xi.b = (Vector9() << 0.01, -0.02, 0.03, -0.1, 0.05, 0.0, 0.0, 0.1, -0.05)
             .finished();
  return xi;
}

TGElement makeElement(const Rot3& R, const Vector3& v, const Vector3& p,
                      const Vector9& a) {
  Eigen::Matrix<double, 3, 2> vp;
  vp.col(0) = v;
  vp.col(1) = p;
  return TGElement(ExtendedPose3<2>(R, vp), a);
}

TGElement makeX() {
  return makeElement(
      Rot3::Rz(0.4) * Rot3::Rx(0.2), Vector3(1.0, -2.0, 0.5),
      Vector3(0.3, 0.1, -0.4),
      (Vector9() << 0.01, -0.02, 0.03, -0.1, 0.05, 0.0, 0.0, 0.1, -0.05)
          .finished());
}

TGElement makeY() {
  return makeElement(
      Rot3::Ry(0.3) * Rot3::Rz(-0.1), Vector3(-0.5, 1.0, 2.0),
      Vector3(0.2, -0.3, 0.1),
      (Vector9() << 0.05, 0.0, -0.05, 0.1, -0.1, 0.2, -0.1, 0.0, 0.05)
          .finished());
}

Input makeU() {
  Input u;
  u.w = Vector3(0.2, -0.1, 0.05);
  u.a = Vector3(-0.05, 0.1, 0.0);
  u.v = Vector3(0.0, 0.0, 9.81);
  u.tau_w = Vector3(0.01, -0.02, 0.0);
  u.tau_a = Vector3(0.02, 0.0, 0.01);
  u.tau_v = Vector3(0.0, 0.03, -0.01);
  u.g_vec = Vector3(0.0, 0.0, -9.81);
  return u;
}

/// Pack an Input into a comparable vector, gravity included.
Eigen::Matrix<double, 21, 1> pack(const Input& u) {
  Eigen::Matrix<double, 21, 1> out;
  out << u.w, u.a, u.v, u.tau_w, u.tau_a, u.tau_v, u.g_vec;
  return out;
}

/// Lambda(phi(X, xi), psi(X, u)) - Ad_{X^-1} Lambda(xi, u), the lift condition
/// residual: zero exactly when the lift is equivariant.
Eigen::Matrix<double, 18, 1> liftConditionResidual(const TGElement& X,
                                                   const State& xi,
                                                   const Input& u) {
  const Eigen::Matrix<double, 18, 1> lhs = Lift(InputOrbit(u)(X))(phi(X, xi));
  const Eigen::Matrix<double, 18, 1> rhs =
      traits<TGElement>::AdjointMap(X.inverse()) * Lift(u)(xi);
  return lhs - rhs;
}

}  // namespace fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace closed_form {

// Lift::operator() evaluates a closed form. This checks both slots against the
// literal definition, Lambda_1 = (W - B + N) + T^-1 (G - N) T and
// Lambda_2 = [B, Lambda_1] - tau, built here from 5x5 matrices only.
TEST(Lift, ClosedFormMatchesMatrixConstruction) {
  const State xi = fixture::makeXi();
  const Input u = fixture::makeU();

  using Se23 = ExtendedPose3<2>;

  const Eigen::Matrix<double, 5, 5> T = xi.T_matrix();
  Vector9 w;
  w.segment<3>(0) = u.w;
  w.segment<3>(3) = u.a;
  w.segment<3>(6) = u.v;
  const Vector9& b = xi.b;
  Vector9 tau;
  tau.segment<3>(0) = u.tau_w;
  tau.segment<3>(3) = u.tau_a;
  tau.segment<3>(6) = u.tau_v;
  Vector9 g = Vector9::Zero();
  g.segment<3>(3) = u.g_vec;

  Eigen::Matrix<double, 5, 5> N = Eigen::Matrix<double, 5, 5>::Zero();
  N(3, 4) = 1.0;

  const Eigen::Matrix<double, 5, 5> L1 =
      Se23::Hat(w) - Se23::Hat(b) + N + T.inverse() * (Se23::Hat(g) - N) * T;
  const Eigen::Matrix<double, 5, 5> B = Se23::Hat(b);
  const Vector9 expected_1 = Se23::Vee(L1);
  const Vector9 expected_2 = Se23::Vee(B * L1 - L1 * B - Se23::Hat(tau));

  const Eigen::Matrix<double, 18, 1> Lambda = Lift(u)(xi);
  EXPECT(fixture::veq(expected_1, Lambda.head<9>()));
  EXPECT(fixture::veq(expected_2, Lambda.tail<9>()));
}

// The position-rate slot carries dp = v. With the virtual input cancelling its
// bias the slot is exactly R^T v.
TEST(Lift, PositionRateCarriesBodyVelocity) {
  State xi = fixture::makeXi();
  Input u = fixture::makeU();
  u.v = xi.b.segment<3>(6);

  EXPECT(fixture::veq(xi.T.rotation().unrotate(xi.T.x(0)),
                      Lift(u)(xi).segment<3>(6), fixture::kTolL));
}

}  // namespace closed_form
/* ************************************************************************* */

/* ************************************************************************* */
namespace equivariance {

// The lift condition, at the identity and at group elements reached both by
// hand and through Expmap. This is what lets the filter linearize once at the
// origin instead of tracking the estimate.
TEST(Lift, SatisfiesTheLiftCondition) {
  const State xi = fixture::makeXi();
  const Input u = fixture::makeU();

  Eigen::Matrix<double, 18, 1> logX;
  logX << 0.15, -0.1, 0.05, 1.0, -2.0, 0.5, 0.3, 0.1, -0.4, 0.1, -0.1, 0.05,
      -0.2, 0.3, 0.0, 0.0, 0.1, -0.1;

  for (const TGElement& X :
       {TGElement::Identity(), fixture::makeX(), TGElement::Expmap(logX)}) {
    EXPECT(fixture::veq(Eigen::Matrix<double, 18, 1>::Zero(),
                        fixture::liftConditionResidual(X, xi, u),
                        fixture::kTolL));
  }
}

// The analytic lift differential is the true derivative in the State chart.
TEST(Lift, JacobianMatchesNumerical) {
  const State xi = fixture::makeXi();
  const Lift lift(fixture::makeU());

  Eigen::Matrix<double, 18, 18> H;
  lift(xi, &H);

  const Eigen::Matrix<double, 18, 18> H_num =
      numericalDerivative11<Eigen::Matrix<double, 18, 1>, State>(
          [&lift](const State& x) { return lift(x); }, xi);
  EXPECT(assert_equal((Matrix)H, (Matrix)H_num, 1e-5));
}

}  // namespace equivariance
/* ************************************************************************* */

/* ************************************************************************* */
namespace input_action {

// psi is a right action on the input space, the axiom that makes the lift
// condition composable across group elements.
TEST(InputOrbit, IsARightAction) {
  const Input u = fixture::makeU();
  const TGElement X = fixture::makeX(), Y = fixture::makeY();

  EXPECT(fixture::veq(fixture::pack(u),
                      fixture::pack(InputOrbit(u)(TGElement::Identity()))));
  EXPECT(fixture::veq(fixture::pack(InputOrbit(u)(X * Y)),
                      fixture::pack(InputOrbit(InputOrbit(u)(X))(Y)),
                      fixture::kTolL));
}

// Gravity is a fixed frame quantity, not part of the input the group acts on,
// so it must pass through untouched.
TEST(InputOrbit, LeavesGravityUntouched) {
  const Input u = fixture::makeU();
  EXPECT(fixture::veq(u.g_vec, InputOrbit(u)(fixture::makeX()).g_vec));
}

}  // namespace input_action
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
