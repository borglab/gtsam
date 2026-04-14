/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1-------------------------------------------
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
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace gtsam;

constexpr double kTol = 1e-9;

struct Rot3VectorAction : public GroupAction<Rot3VectorAction, Rot3, Vector3> {
  static constexpr ActionType type = ActionType::Left;

  Vector3 operator()(const Rot3& R, const Vector3& t,
                     OptionalJacobian<3, 3> HR = {},
                     OptionalJacobian<3, 3> Ht = {}) const {
    return R.rotate(t, HR, Ht);
  }

  template <typename ProductType>
  static ProductType Expmap(
      const Vector3& w, const Vector3& rho,
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = {},
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = {}) {
    Matrix6 Hpose;
    Vector6 xi;
    xi << w, rho;
    const Pose3 pose = Pose3::Expmap(xi, H1 || H2 ? &Hpose : nullptr);
    if (H1) *H1 = Hpose.leftCols<3>();
    if (H2) *H2 = Hpose.rightCols<3>();
    return ProductType(pose.rotation(), pose.translation());
  }

  template <typename ProductType>
  static Vector6 Logmap(const ProductType& p,
                        OptionalJacobian<6, 6> H = {}) {
    Matrix6 Hpose;
    const Vector6 xi =
        Pose3::Logmap(Pose3(p.first, p.second), H ? &Hpose : nullptr);
    if (H) *H = Hpose;
    return xi;
  }

};

using Semidirect = ProductLieGroup<Rot3, Vector3, Rot3VectorAction>;

namespace {

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

}  // namespace

/* ************************************************************************* */
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

/* ************************************************************************* */
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

/* ************************************************************************* */
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

/* ************************************************************************* */
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

/* ************************************************************************* */
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

/* ************************************************************************* */
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

/* ************************************************************************* */
// Check that the semidirect adjoint matches the Pose3 adjoint.
TEST(testActionProduct, AdjointMap) {
  const Semidirect state = semidirectState4();

  EXPECT(assert_equal(asPose3(state).AdjointMap(), state.AdjointMap(), kTol));
}

/* ************************************************************************* */
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

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
