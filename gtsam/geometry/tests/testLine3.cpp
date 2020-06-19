#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Line3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Line3)
GTSAM_CONCEPT_MANIFOLD_INST(Line3)

static const Line3 l(Rot3(), 1, 1);

// Testing equals function of Line3
TEST(Line3, equals) {
  Line3 l_same = l;
  EXPECT(l.equals(l_same));
  Line3 l2(Rot3(), 1, 2);
  EXPECT(!l.equals(l2));
}

// testing localCoordinates along 4 dimensions
TEST(Line3, localCoordinates) {
  // l1 and l differ only in a_
  Line3 l1(Rot3(), 2, 1);
  Vector4 v1(0, 0, -1, 0);
  CHECK(assert_equal(l1.localCoordinates(l), v1));

  // l2 and l differ only in b_
  Line3 l2(Rot3(), 1, 2);
  Vector4 v2(0, 0, 0, -1);
  CHECK(assert_equal(l2.localCoordinates(l), v2));

  // l3 and l differ in R_x
  Rot3 r3 = Rot3::Expmap(Vector3(M_PI / 4, 0, 0));
  Line3 l3(r3, 1, 1);
  Vector4 v3(-M_PI / 4, 0, 0, 0);
  CHECK(assert_equal(l3.localCoordinates(l), v3));

  // l4 and l differ in R_y
  Rot3 r4 = Rot3::Expmap(Vector3(0, M_PI / 4, 0));
  Line3 l4(r4, 1, 1);
  Vector4 v4(0, -M_PI / 4, 0, 0);
  CHECK(assert_equal(l4.localCoordinates(l), v4));

  // Jacobians
  Line3 l5(Rot3::Expmap(Vector3(M_PI / 3, -M_PI / 4, 0)), -0.8, 2);
  Values val;
  val.insert(1, l);
  val.insert(2, l5);
  Line3_ l_(1);
  Line3_ l5_(2);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(4, 0.1);
  Vector4_ local_(l5_, &Line3::localCoordinates, l_);
  ExpressionFactor<Vector4> f(model, l5.localCoordinates(l), local_);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, val, 1e-5, 1e-7);
}

// testing retract along 4 dimensions
TEST(Line3, retract) {
  // l1 and l differ only in a_
  Vector4 v1(0, 0, 0, 1);
  Line3 l1(Rot3(), 1, 2);
  EXPECT(l1.equals(l.retract(v1)));

  // l2 and l differ only in b_
  Vector4 v2(0, 0, 1, 0);
  Line3 l2(Rot3(), 2, 1);
  EXPECT(l2.equals(l.retract(v2)));

  // l3 and l differ in R_x
  Vector4 v3(M_PI / 4, 0, 0, 0);
  Rot3 r3;
  r3 = r3.Expmap(Vector3(M_PI / 4, 0, 0));
  Line3 l3(r3, 1, 1);
  EXPECT(l3.equals(l.retract(v3)));

  // l4 and l differ in R_y
  Vector4 v4(0, M_PI / 4, 0, 0);
  Rot3 r4 = Rot3::Expmap(Vector3(0, M_PI / 4, 0));
  Line3 l4(r4, 1, 1);
  EXPECT(l4.equals(l.retract(v4)));

  // Jacobians
  Vector4 v5(M_PI / 3, -M_PI / 4, -0.4, 1.2); // arbitrary vector
  Values val;
  val.insert(1, l);
  val.insert(2, v5);
  Line3_ l_(1);
  Vector4_ v5_(2);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(4, 0.1);
  Line3_ retract_(l_, &Line3::retract, v5_);
  ExpressionFactor<Line3> f(model, l.retract(v5), retract_);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, val, 1e-5, 1e-7);
}

// testing manifold property - Retract(p, Local(p,q)) == q
TEST(Line3, retractOfLocalCoordinates) {
  Rot3 r2 = Rot3::Expmap(Vector3(M_PI / 4, M_PI / 3, 0));
  Line3 l2(r2, 5, 9);
  EXPECT(assert_equal(l.retract(l.localCoordinates(l2)), l2));
}

// testing manifold property - Local(p, Retract(p, v)) == v
TEST(Line3, localCoordinatesOfRetract) {
  Vector4 r2(2.3, 0.987, -3, 4);
  EXPECT(assert_equal(l.localCoordinates(l.retract(r2)), r2));
}

// transform from world to camera test
TEST(Line3, transformToExpressionJacobians) {
  Rot3 r = Rot3::Expmap(Vector3(0, M_PI / 3, 0));
  Vector3 t(0, 0, 0);
  Pose3 p(r, t);

  Line3 l_c(r.inverse(), 1, 1);
  Line3 l_w(Rot3(), 1, 1);
  EXPECT(l_c.equals(transformTo(p, l_w)));

  Line3_ l_(1);
  Pose3_ p_(2);
  Values val;
  val.insert(1, l_w);
  val.insert(2, p);

  SharedNoiseModel model = noiseModel::Isotropic::Sigma(4, 0.1);
  ExpressionFactor<Line3> f(model, transformTo(p, l_w), transformTo(p_, l_));
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, val, 1e-5, 1e-7);
}

// projection in camera frame test
TEST(Line3, projection) {
  Rot3 r = Rot3::Expmap(Vector3(0, 0, 0));
  Line3 wL(r, 1, 1);

  Unit3 expected = Unit3::FromPoint3(Point3(-1, 1, 0));
  EXPECT(expected.equals(wL.project()));

  Values val;
  val.insert(1, wL);
  Line3_ wL_(1);

  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.1);
  Unit3_ projected_(wL_, &Line3::project);
  ExpressionFactor<Unit3> f(model, expected, projected_);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, val, 1e-5, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
