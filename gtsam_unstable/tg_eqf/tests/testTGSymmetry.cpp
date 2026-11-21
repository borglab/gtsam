/**
 * @file  testTGSymmetry.cpp
 * @brief Action axioms for phi, and the Jacobians of the Orbit and
 *        Diffeomorphism functors in the State chart.
 *
 * Layer above testTGState.cpp and testTGGroup.cpp: how the group acts on the
 * manifold. The Jacobians here are read in the State chart, so they are the
 * first place the chart choice becomes visible.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/tg_eqf/TGGeometry.h>

using namespace gtsam::tgeqf;
using namespace gtsam;

/* ************************************************************************* */
namespace fixture {

constexpr double kTol = 1e-9;
constexpr double kTolL = 1e-7;

bool seq(const State& a, const State& b, double tol = kTol) {
  return traits<State>::Equals(a, b, tol);
}

bool meq(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
         double tol = kTol) {
  return assert_equal((Matrix)a, (Matrix)b, tol);
}

State makeState(const Rot3& R, const Vector3& v, const Vector3& p,
                const Vector9& b) {
  Eigen::Matrix<double, 3, 2> vp;
  vp.col(0) = v;
  vp.col(1) = p;
  State xi;
  xi.T = ExtendedPose3<2>(R, vp);
  xi.b = b;
  return xi;
}

State makeXi() {
  return makeState(
      Rot3::Rz(0.3) * Rot3::Rx(0.1), Vector3(1.0, -2.0, 0.5),
      Vector3(0.3, 0.1, -0.4),
      (Vector9() << 0.01, -0.02, 0.03, -0.1, 0.05, 0.0, 0.0, 0.1, -0.05)
          .finished());
}

/// A second state, unrelated to makeXi().
State makeXiEst() {
  return makeState(
      Rot3::Ry(-0.2) * Rot3::Rz(0.5), Vector3(-0.7, 0.4, 1.2),
      Vector3(2.0, -1.5, 0.6),
      (Vector9() << -0.03, 0.01, 0.02, 0.2, -0.15, 0.07, 0.05, 0.0, -0.02)
          .finished());
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
      (Vector9() << 0.1, -0.1, 0.05, -0.2, 0.3, 0.0, 0.0, 0.1, -0.1)
          .finished());
}

TGElement makeY() {
  return makeElement(
      Rot3::Ry(0.3) * Rot3::Rz(-0.1), Vector3(-0.5, 1.0, 2.0),
      Vector3(0.2, -0.3, 0.1),
      (Vector9() << 0.05, 0.0, -0.05, 0.1, -0.1, 0.2, -0.1, 0.0, 0.05)
          .finished());
}

Eigen::Matrix<double, 18, 18> orbitJacobian(const TGElement& X,
                                            const State& xi_ref) {
  Eigen::Matrix<double, 18, 18> H;
  const TGSymmetry::Orbit orbit(xi_ref);
  orbit(X, &H);
  return H;
}

Eigen::Matrix<double, 18, 18> diffeoJacobian(const TGElement& X,
                                             const State& xi) {
  Eigen::Matrix<double, 18, 18> H;
  const TGSymmetry::Diffeomorphism diffeo(X);
  diffeo(xi, &H);
  return H;
}

}  // namespace fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace action_axioms {

// phi is a right action: the identity acts trivially and phi(XY, .) is phi(X,.)
// followed by phi(Y, .). The fiber term is what makes the order matter.
TEST(Phi, IsARightAction) {
  const State xi = fixture::makeXi();
  const TGElement X = fixture::makeX(), Y = fixture::makeY();

  EXPECT(fixture::seq(phi(TGElement::Identity(), xi), xi));
  EXPECT(fixture::seq(phi(X * Y, xi), phi(Y, phi(X, xi)), fixture::kTolL));
  EXPECT(fixture::seq(phi(X.inverse(), phi(X, xi)), xi, fixture::kTolL));
}

// On the navigation block the action is right translation of the extended pose,
// T_out = T * A. Checked in the 5x5 matrix representation, which reaches
// State::T_matrix and ExtendedPose3<2>::matrix() from an independent
// direction.
TEST(Phi, RightTranslatesTheExtendedPose) {
  const State xi = fixture::makeXi();
  const TGElement X = fixture::makeX();
  EXPECT(fixture::meq(phi(X, xi).T_matrix(), xi.T_matrix() * X.first.matrix(),
                      fixture::kTolL));
}

// The action is transitive and free: phiInverse returns the one element taking
// xi_ref to xi_est, and it recovers the element phi was called with.
TEST(PhiInverse, InvertsTheOrbitMap) {
  const State xi_ref = fixture::makeXi();
  const State xi_est = fixture::makeXiEst();
  const TGElement X = fixture::makeX();

  EXPECT(fixture::seq(phi(phiInverse(xi_ref, xi_est), xi_ref), xi_est,
                      fixture::kTolL));
  EXPECT(traits<TGElement>::Equals(phiInverse(xi_ref, phi(X, xi_ref)), X,
                                   fixture::kTolL));
}

}  // namespace action_axioms
/* ************************************************************************* */

/* ************************************************************************* */
namespace functors {

// The Orbit and Diffeomorphism functors are the two partial applications of the
// same phi, so their values must agree with it.
TEST(Symmetry, FunctorsAgreeWithPhi) {
  const State xi = fixture::makeXi();
  const TGElement X = fixture::makeX();

  const TGSymmetry::Orbit orbit(xi);
  const TGSymmetry::Diffeomorphism diffeo(X);
  EXPECT(fixture::seq(orbit(X), phi(X, xi)));
  EXPECT(fixture::seq(diffeo(xi), phi(X, xi)));
}

// Diffeomorphism's Jacobian is AdjointMap(X^-1), a genuinely computed value,
// so check it against an independent finite difference of phi.
//
// Orbit's Jacobian is not checked this way: phi(Exp(dx), y0) is, by the
// chart's own definition (Local = Log . phiInverse, with phiInverse the exact
// inverse of phi in its group argument), always exactly dx away from y0 in
// State's own chart, for every X and xi -- not approximately. A finite
// difference against phi would reproduce Identity regardless of what
// Orbit::operator() returns, so it cannot distinguish a correct
// implementation from a broken one; see PhiInverse.InvertsTheOrbitMap above
// for the underlying axiom.
TEST(Diffeomorphism, JacobianMatchesNumerical) {
  const State xi = fixture::makeXi();
  const TGElement X = fixture::makeX();

  const Eigen::Matrix<double, 18, 18> diffeo_num =
      numericalDerivative11<State, State>(
          [&X](const State& x) { return phi(X, x); }, xi);
  EXPECT(fixture::meq(fixture::diffeoJacobian(X, xi), diffeo_num, 1e-5));
}

// Orbit::operator() returns the hardcoded Identity for H; check it directly
// rather than through a numerical derivative (see the comment above).
TEST(Orbit, JacobianIsIdentity) {
  for (const State& xi_ref : {State::identity(), fixture::makeXi()}) {
    for (const TGElement& X : {TGElement::Identity(), fixture::makeX()}) {
      EXPECT(fixture::meq(fixture::orbitJacobian(X, xi_ref),
                          Eigen::Matrix<double, 18, 18>::Identity()));
    }
  }
}

// Differentiating phi(X^-1, phi(X, .)) = id gives back the identity, so the two
// Diffeomorphism Jacobians are mutual inverses. This constrains which side the
// adjoint acts on, which a value check at a single point does not.
TEST(Diffeomorphism, JacobianInvertsThatOfTheInverseAction) {
  const State xi = fixture::makeXi();
  const TGElement X = fixture::makeX();

  EXPECT(fixture::meq(fixture::diffeoJacobian(X.inverse(), phi(X, xi)) *
                          fixture::diffeoJacobian(X, xi),
                      Eigen::Matrix<double, 18, 18>::Identity(),
                      fixture::kTolL));
}

}  // namespace functors
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
