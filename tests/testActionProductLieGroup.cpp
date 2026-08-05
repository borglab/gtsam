/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * --------------------------------------------------------------------------
 */

/**
 * @file testActionProductLieGroup.cpp
 * @date April, 2026
 * @author Rohan Bansal
 * @author Jennifer Oum
 * @brief unit tests for action-parameterized product Lie groups
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>

using namespace gtsam;

/* ************************************************************************* */
namespace semidirect_product_fixture {

constexpr double kTol = 1e-9;

// Rot2 acting on Point2 by rotation: φ(R, t) = R·t.
// The infinitesimal generator is Aφ(u) = Hat(u), a 2x2 skew matrix.
struct Rot2PointAction : public GroupAction<Rot2PointAction, Rot2, Point2> {
  static constexpr ActionType type = ActionType::Left;

  Point2 operator()(const Rot2& R, const Point2& t,
                    OptionalJacobian<2, 1> HR = {},
                    OptionalJacobian<2, 2> Ht = {}) const {
    return R.rotate(t, HR, Ht);
  }

  static Matrix2 generator(const Vector1& u) { return Rot2::Hat(u); }
};

// Rot3 acting on Vector3 by rotation: φ(R, t) = R·t.
// The infinitesimal generator Aφ(u)·t = d/dt(exp(tu∧)·t)|₀ = u∧·t,
// so generator(u) = u∧ (skew-symmetric matrix). ProductLieGroup derives
// Expmap and Logmap automatically via φ₁(u∧) = SO(3) left Jacobian.
struct Rot3VectorAction : public GroupAction<Rot3VectorAction, Rot3, Vector3> {
  static constexpr ActionType type = ActionType::Left;

  Vector3 operator()(const Rot3& R, const Vector3& t,
                     OptionalJacobian<3, 3> HR = {},
                     OptionalJacobian<3, 3> Ht = {}) const {
    return R.rotate(t, HR, Ht);
  }

  static Matrix3 generator(const Vector3& u) { return skewSymmetric(u); }
};

using Semidirect2 = ProductLieGroup<Rot2, Point2, Rot2PointAction>;
using Semidirect = ProductLieGroup<Rot3, Vector3, Rot3VectorAction>;

Semidirect2 semidirect2State() {
  return Semidirect2(Rot2::fromAngle(0.35), Point2(0.4, -0.6));
}

Vector3 semidirect2Xi() {
  Vector3 xi;
  xi << 0.25, 0.3, -0.2;
  return xi;
}

Vector3 pose2XiFromSemidirect2(const Vector3& xi) {
  Vector3 pose2Xi;
  pose2Xi << xi(1), xi(2), xi(0);
  return pose2Xi;
}

Vector3 semidirect2XiFromPose2(const Vector3& xi) {
  Vector3 semidirectXi;
  semidirectXi << xi(2), xi(0), xi(1);
  return semidirectXi;
}

Semidirect2 expmapSemidirect2Proxy(const Vector3& vec) {
  return Semidirect2::Expmap(vec);
}

Vector3 logmapSemidirect2Proxy(const Semidirect2& p) {
  return Semidirect2::Logmap(p);
}

Pose2 asPose2(const Semidirect2& state) {
  return Pose2(state.first, state.second);
}

Semidirect semidirectState1() {
  return Semidirect(Rot3::RzRyRx(0.1, 0.2, -0.3), Vector3(1.0, -0.5, 0.25));
}

Semidirect semidirectState2() {
  return Semidirect(Rot3::RzRyRx(-0.2, 0.1, 0.15), Vector3(-0.75, 0.4, 1.2));
}

Semidirect semidirectState3() {
  return Semidirect(Rot3::RzRyRx(0.2, -0.1, 0.05), Vector3(0.3, -0.6, 0.8));
}

Semidirect semidirectState4() {
  return Semidirect(Rot3::RzRyRx(0.1, -0.2, 0.3), Vector3(0.4, -0.1, 0.2));
}

Vector6 semidirectXi() {
  Vector6 xi;
  xi << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2;
  return xi;
}

Vector6 retractDelta() {
  Vector6 delta;
  delta << 0.05, -0.04, 0.03, 0.1, -0.2, 0.05;
  return delta;
}

Semidirect composeSemidirectProxy(const Semidirect& A, const Semidirect& B) {
  return A.compose(B);
}

Semidirect betweenSemidirectProxy(const Semidirect& A, const Semidirect& B) {
  return A.between(B);
}

Semidirect inverseSemidirectProxy(const Semidirect& A) { return A.inverse(); }

Semidirect expmapSemidirectProxy(const Vector6& vec) {
  return Semidirect::Expmap(vec);
}

Vector6 logmapSemidirectProxy(const Semidirect& p) {
  return Semidirect::Logmap(p);
}

Semidirect retractSemidirectProxy(const Semidirect& X, const Vector6& v) {
  return X.retract(v);
}

Vector6 localCoordinatesSemidirectProxy(const Semidirect& X,
                                        const Semidirect& Y) {
  return X.localCoordinates(Y);
}

Pose3 asPose3(const Semidirect& state) {
  return Pose3(state.first, state.second);
}

// A second semidirect instance, Rot2 ⋉ R², checks the generic kernel path in
// a different dimension with Pose2 as the oracle.
TEST(Lie, ProductLieGroupSemidirectAction2D) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Semidirect2>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Semidirect2>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<Semidirect2>);

  const Rot2PointAction action;
  const Rot2 R1 = Rot2::fromAngle(0.3);
  const Rot2 R2 = Rot2::fromAngle(-0.2);
  const Point2 t(0.4, -0.5);
  EXPECT_LEFT_ACTION(action, R1, R2, t);

  const Vector1 u = (Vector1() << 0.35).finished();
  const Point2 p(0.2, -0.7);
  const double eps = 1e-7;
  const Point2 generatorAction = Rot2PointAction::generator(u) * p;
  const Point2 generatorFiniteDifference =
      (Rot2::Expmap((Vector1() << eps * u(0)).finished()).rotate(p) - p) / eps;
  EXPECT(assert_equal(generatorAction, generatorFiniteDifference, 1e-6));

  const Vector3 xi = semidirect2Xi();
  const Semidirect2 actual = Semidirect2::Expmap(xi);
  EXPECT(assert_equal(Pose2::Expmap(pose2XiFromSemidirect2(xi)),
                      asPose2(actual), kTol));
  EXPECT(assert_equal(
      xi, semidirect2XiFromPose2(Pose2::Logmap(asPose2(actual))), kTol));
}

// Check Expmap/Logmap values and Jacobians for a second semidirect product,
// independent of the Pose3-specific ordering conventions.
TEST(testActionProduct, ExpmapLogmap2D) {
  const Vector3 xi = semidirect2Xi();

  Matrix expH;
  const Semidirect2 actual = Semidirect2::Expmap(xi, expH);
  const Matrix numericExpH = numericalDerivative11(expmapSemidirect2Proxy, xi);

  EXPECT(assert_equal(Pose2::Expmap(pose2XiFromSemidirect2(xi)),
                      asPose2(actual), kTol));
  EXPECT(assert_equal(numericExpH, expH, 1e-6));

  const Semidirect2 state = semidirect2State();
  Matrix logH;
  const Vector3 actualLog = Semidirect2::Logmap(state, logH);
  const Matrix numericLogH =
      numericalDerivative11(logmapSemidirect2Proxy, state);

  EXPECT(assert_equal(semidirect2XiFromPose2(Pose2::Logmap(asPose2(state))),
                      actualLog, kTol));
  EXPECT(assert_equal(numericLogH, logH, 1e-6));
}

// Verify the semidirect product obeys the left action law and matches Pose3
// behavior.
TEST(Lie, ProductLieGroupSemidirectAction) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Semidirect>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Semidirect>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<Semidirect>);

  const Rot3VectorAction action;
  const Rot3 R1 = Rot3::RzRyRx(0.1, -0.2, 0.3);
  const Rot3 R2 = Rot3::RzRyRx(-0.2, 0.05, 0.1);
  const Vector3 t(0.4, -0.5, 0.6);
  EXPECT_LEFT_ACTION(action, R1, R2, t);

  const Semidirect identity;
  const Vector6 xi = semidirectXi();
  const Semidirect actual = identity.expmap(xi);
  EXPECT(assert_equal(Pose3::Expmap(xi), asPose3(actual), kTol));
  EXPECT(assert_equal(xi, identity.logmap(actual), kTol));

  const Semidirect a =
      Semidirect(Rot3::RzRyRx(0.1, 0.2, -0.1), Vector3(1.0, -2.0, 0.5));
  const Semidirect b =
      Semidirect(Rot3::RzRyRx(-0.3, 0.15, 0.2), Vector3(-0.25, 0.4, 1.5));
  const Semidirect c = semidirectState3();
  EXPECT(assert_equal(asPose3((a * b) * c), asPose3(a * (b * c)), kTol));
  EXPECT(assert_equal(asPose3(a * a.inverse()), Pose3(), kTol));
  EXPECT(assert_equal(asPose3(a * b), asPose3(a) * asPose3(b), kTol));
}

// Check semidirect compose values and Jacobians against Pose3 and numerical
// derivatives.
TEST(testActionProduct, compose) {
  const Semidirect state1 = semidirectState1();
  const Semidirect state2 = semidirectState2();

  Matrix actH1, actH2;
  const Semidirect actual = state1.compose(state2, actH1, actH2);
  const Matrix numericH1 =
      numericalDerivative21(composeSemidirectProxy, state1, state2);
  const Matrix numericH2 =
      numericalDerivative22(composeSemidirectProxy, state1, state2);

  EXPECT(
      assert_equal(asPose3(actual), asPose3(state1) * asPose3(state2), kTol));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
  EXPECT(assert_equal(numericH2, actH2, 1e-6));
}

// Check semidirect between values and Jacobians against Pose3 and numerical
// derivatives.
TEST(testActionProduct, between) {
  const Semidirect state1 = semidirectState1();
  const Semidirect state2 = semidirectState2();

  Matrix actH1, actH2;
  const Semidirect actual = state1.between(state2, actH1, actH2);
  const Matrix numericH1 =
      numericalDerivative21(betweenSemidirectProxy, state1, state2);
  const Matrix numericH2 =
      numericalDerivative22(betweenSemidirectProxy, state1, state2);

  EXPECT(assert_equal(asPose3(actual), asPose3(state1).between(asPose3(state2)),
                      kTol));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
  EXPECT(assert_equal(numericH2, actH2, 1e-6));
}

// Check the semidirect inverse and its Jacobian against Pose3 and numerical
// derivatives.
TEST(testActionProduct, inverse) {
  const Semidirect state =
      Semidirect(Rot3::RzRyRx(0.2, -0.1, 0.05), Vector3(0.5, -1.2, 0.8));

  Matrix actH;
  const Semidirect actual = state.inverse(actH);
  const Matrix numericH = numericalDerivative11(inverseSemidirectProxy, state);

  EXPECT(assert_equal(asPose3(actual), asPose3(state).inverse(), kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
}

// Check the semidirect Expmap and its Jacobian against Pose3 and numerical
// derivatives.
TEST(testActionProduct, Expmap) {
  const Vector6 xi = semidirectXi();

  Matrix actH;
  const Semidirect actual = Semidirect::Expmap(xi, actH);
  const Matrix numericH = numericalDerivative11(expmapSemidirectProxy, xi);

  EXPECT(assert_equal(Pose3::Expmap(xi), asPose3(actual), kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
}

// Check the semidirect Logmap and its Jacobian against Pose3 and numerical
// derivatives.
TEST(testActionProduct, Logmap) {
  const Semidirect state = semidirectState4();

  Matrix actH;
  const Vector6 actual = Semidirect::Logmap(state, actH);
  const Matrix numericH = numericalDerivative11(logmapSemidirectProxy, state);

  EXPECT(assert_equal(Pose3::Logmap(asPose3(state)), actual, kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
}

// Check that the semidirect adjoint matches the Pose3 adjoint.
TEST(testActionProduct, AdjointMap) {
  const Semidirect state = semidirectState4();

  EXPECT(assert_equal(asPose3(state).AdjointMap(), state.AdjointMap(), kTol));
}

// Check Expmap-based retract/localCoordinates consistency and both Jacobians
// against numerical derivatives.
TEST(testActionProduct, retractAndLocalCoordinates) {
  const Semidirect state = semidirectState4();
  const Vector6 delta = retractDelta();

  Matrix retractH1, retractH2, localH1, localH2;
  const Semidirect updated = state.retract(delta, retractH1, retractH2);
  const Vector6 recovered = state.localCoordinates(updated, localH1, localH2);

  EXPECT(assert_equal(asPose3(updated),
                      asPose3(state).compose(Pose3::Expmap(delta)), kTol));
  EXPECT(assert_equal(delta, recovered, kTol));

  const Matrix numericRetractH1 =
      numericalDerivative21(retractSemidirectProxy, state, delta);
  const Matrix numericRetractH2 =
      numericalDerivative22(retractSemidirectProxy, state, delta);
  const Matrix numericLocalH1 =
      numericalDerivative21(localCoordinatesSemidirectProxy, state, updated);
  const Matrix numericLocalH2 =
      numericalDerivative22(localCoordinatesSemidirectProxy, state, updated);

  EXPECT(assert_equal(numericRetractH1, retractH1, 1e-6));
  EXPECT(assert_equal(numericRetractH2, retractH2, 1e-6));
  EXPECT(assert_equal(numericLocalH1, localH1, 1e-6));
  EXPECT(assert_equal(numericLocalH2, localH2, 1e-6));
}

}  // namespace semidirect_product_fixture
/* ************************************************************************* */

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
