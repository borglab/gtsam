/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testUnit3.cpp
 * @date Feb 03, 2012
 * @author Can Erdogan
 * @author Frank Dellaert
 * @author Alex Trevor
 * @brief Tests the Unit3 class
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>


#include <CppUnitLite/TestHarness.h>

#include <cmath>
#include <random>

using namespace std::placeholders;
using namespace gtsam;
using namespace std;
using gtsam::symbol_shorthand::U;

GTSAM_CONCEPT_TESTABLE_INST(Unit3)
GTSAM_CONCEPT_MANIFOLD_INST(Unit3)

//*******************************************************************************
Point3 point3_(const Unit3& p) {
  return p.point3();
}

TEST(Unit3, point3) {
  const vector<Point3> ps{Point3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1),
                          Point3(1, 1, 0) / sqrt(2.0)};
  Matrix actualH, expectedH;
  for(Point3 p: ps) {
    Unit3 s(p);
    expectedH = numericalDerivative11<Point3, Unit3>(point3_, s);
    EXPECT(assert_equal(p, s.point3(actualH), 1e-5));
    EXPECT(assert_equal(expectedH, actualH, 1e-5));
  }
}

//*******************************************************************************
static Unit3 rotate_(const Rot3& R, const Unit3& p) {
  return R * p;
}

TEST(Unit3, rotate) {
  Rot3 R = Rot3::Yaw(0.5);
  Unit3 p(1, 0, 0);
  Unit3 expected = Unit3(R.column(1));
  Unit3 actual = R * p;
  EXPECT(assert_equal(expected, actual, 1e-5));
  Matrix actualH, expectedH;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expectedH = numericalDerivative21(rotate_, R, p);
    R.rotate(p, actualH, boost::none);
    EXPECT(assert_equal(expectedH, actualH, 1e-5));
  }
  {
    expectedH = numericalDerivative22(rotate_, R, p);
    R.rotate(p, boost::none, actualH);
    EXPECT(assert_equal(expectedH, actualH, 1e-5));
  }
}

//*******************************************************************************
static Unit3 unrotate_(const Rot3& R, const Unit3& p) {
  return R.unrotate(p);
}

TEST(Unit3, unrotate) {
  Rot3 R = Rot3::Yaw(-M_PI / 4.0);
  Unit3 p(1, 0, 0);
  Unit3 expected = Unit3(1, 1, 0);
  Unit3 actual = R.unrotate(p);
  EXPECT(assert_equal(expected, actual, 1e-5));

  Matrix actualH, expectedH;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expectedH = numericalDerivative21(unrotate_, R, p);
    R.unrotate(p, actualH, boost::none);
    EXPECT(assert_equal(expectedH, actualH, 1e-5));
  }
  {
    expectedH = numericalDerivative22(unrotate_, R, p);
    R.unrotate(p, boost::none, actualH);
    EXPECT(assert_equal(expectedH, actualH, 1e-5));
  }
}

TEST(Unit3, dot) {
  Unit3 p(1, 0.2, 0.3);
  Unit3 q = p.retract(Vector2(0.5, 0));
  Unit3 r = p.retract(Vector2(0.8, 0));
  Unit3 t = p.retract(Vector2(0, 0.3));
  EXPECT(assert_equal(1.0, p.dot(p), 1e-5));
  EXPECT(assert_equal(0.877583, p.dot(q), 1e-5));
  EXPECT(assert_equal(0.696707, p.dot(r), 1e-5));
  EXPECT(assert_equal(0.955336, p.dot(t), 1e-5));

  // Use numerical derivatives to calculate the expected Jacobians
  Matrix H1, H2;
  std::function<double(const Unit3&, const Unit3&)> f =
      std::bind(&Unit3::dot, std::placeholders::_1, std::placeholders::_2,  //
                boost::none, boost::none);
  {
    p.dot(q, H1, H2);
    EXPECT(assert_equal(numericalDerivative21<double,Unit3>(f, p, q), H1, 1e-5));
    EXPECT(assert_equal(numericalDerivative22<double,Unit3>(f, p, q), H2, 1e-5));
  }
  {
    p.dot(r, H1, H2);
    EXPECT(assert_equal(numericalDerivative21<double,Unit3>(f, p, r), H1, 1e-5));
    EXPECT(assert_equal(numericalDerivative22<double,Unit3>(f, p, r), H2, 1e-5));
  }
  {
    p.dot(t, H1, H2);
    EXPECT(assert_equal(numericalDerivative21<double,Unit3>(f, p, t), H1, 1e-5));
    EXPECT(assert_equal(numericalDerivative22<double,Unit3>(f, p, t), H2, 1e-5));
  }
}

//*******************************************************************************
TEST(Unit3, error) {
  Unit3 p(1, 0, 0), q = p.retract(Vector2(0.5, 0)), //
  r = p.retract(Vector2(0.8, 0));
  EXPECT(assert_equal((Vector)(Vector2(0, 0)), p.error(p), 1e-5));
  EXPECT(assert_equal((Vector)(Vector2(0.479426, 0)), p.error(q), 1e-5));
  EXPECT(assert_equal((Vector)(Vector2(0.717356, 0)), p.error(r), 1e-5));

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalDerivative11<Vector2,Unit3>(
        std::bind(&Unit3::error, &p, std::placeholders::_1, boost::none), q);
    p.error(q, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-5));
  }
  {
    expected = numericalDerivative11<Vector2,Unit3>(
        std::bind(&Unit3::error, &p, std::placeholders::_1, boost::none), r);
    p.error(r, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-5));
  }
}

//*******************************************************************************
TEST(Unit3, error2) {
  Unit3 p(0.1, -0.2, 0.8);
  Unit3 q = p.retract(Vector2(0.2, -0.1));
  Unit3 r = p.retract(Vector2(0.8, 0));

  // Hard-coded as simple regression values
  EXPECT(assert_equal((Vector)(Vector2(0.0, 0.0)), p.errorVector(p), 1e-5));
  EXPECT(assert_equal((Vector)(Vector2(0.198337495, -0.0991687475)), p.errorVector(q), 1e-5));
  EXPECT(assert_equal((Vector)(Vector2(0.717356, 0)), p.errorVector(r), 1e-5));

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalDerivative21<Vector2, Unit3, Unit3>(
        std::bind(&Unit3::errorVector, std::placeholders::_1,
                  std::placeholders::_2, boost::none, boost::none),
        p, q);
    p.errorVector(q, actual, boost::none);
    EXPECT(assert_equal(expected, actual, 1e-5));
  }
  {
    expected = numericalDerivative21<Vector2, Unit3, Unit3>(
        std::bind(&Unit3::errorVector, std::placeholders::_1,
                  std::placeholders::_2, boost::none, boost::none),
        p, r);
    p.errorVector(r, actual, boost::none);
    EXPECT(assert_equal(expected, actual, 1e-5));
  }
  {
    expected = numericalDerivative22<Vector2, Unit3, Unit3>(
        std::bind(&Unit3::errorVector, std::placeholders::_1,
                  std::placeholders::_2, boost::none, boost::none),
        p, q);
    p.errorVector(q, boost::none, actual);
    EXPECT(assert_equal(expected, actual, 1e-5));
  }
  {
    expected = numericalDerivative22<Vector2, Unit3, Unit3>(
        std::bind(&Unit3::errorVector, std::placeholders::_1,
                  std::placeholders::_2, boost::none, boost::none),
        p, r);
    p.errorVector(r, boost::none, actual);
    EXPECT(assert_equal(expected, actual, 1e-5));
  }
}

//*******************************************************************************
TEST(Unit3, distance) {
  Unit3 p(1, 0, 0), q = p.retract(Vector2(0.5, 0)), //
  r = p.retract(Vector2(0.8, 0));
  EXPECT_DOUBLES_EQUAL(0, p.distance(p), 1e-5);
  EXPECT_DOUBLES_EQUAL(0.47942553860420301, p.distance(q), 1e-5);
  EXPECT_DOUBLES_EQUAL(0.71735609089952279, p.distance(r), 1e-5);

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalGradient<Unit3>(
        std::bind(&Unit3::distance, &p, std::placeholders::_1, boost::none), q);
    p.distance(q, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-5));
  }
  {
    expected = numericalGradient<Unit3>(
        std::bind(&Unit3::distance, &p, std::placeholders::_1, boost::none), r);
    p.distance(r, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-5));
  }
}

//*******************************************************************************
TEST(Unit3, localCoordinates0) {
  Unit3 p;
  Vector actual = p.localCoordinates(p);
  EXPECT(assert_equal(Z_2x1, actual, 1e-5));
}

TEST(Unit3, localCoordinates) {
  {
    Unit3 p, q;
    Vector2 expected = Vector2::Zero();
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal((Vector) Z_2x1, actual, 1e-5));
    EXPECT(assert_equal(q, p.retract(expected), 1e-5));
  }
  {
    Unit3 p, q(1, 6.12385e-21, 0);
    Vector2 expected = Vector2::Zero();
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal((Vector) Z_2x1, actual, 1e-5));
    EXPECT(assert_equal(q, p.retract(expected), 1e-5));
  }
  {
    Unit3 p, q(-1, 0, 0);
    Vector2 expected(M_PI, 0);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(expected, actual, 1e-5));
    EXPECT(assert_equal(q, p.retract(expected), 1e-5));
  }
  {
    Unit3 p, q(0, 1, 0);
    Vector2 expected(0,-M_PI_2);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(expected, actual, 1e-5));
    EXPECT(assert_equal(q, p.retract(expected), 1e-5));
  }
  {
    Unit3 p, q(0, -1, 0);
    Vector2 expected(0, M_PI_2);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(expected, actual, 1e-5));
    EXPECT(assert_equal(q, p.retract(expected), 1e-5));
  }
  {
    Unit3 p(0,1,0), q(0,-1,0);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(q, p.retract(actual), 1e-5));
  }
  {
    Unit3 p(0,0,1), q(0,0,-1);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(q, p.retract(actual), 1e-5));
  }

  double twist = 1e-4;
  {
    Unit3 p(0, 1, 0), q(0 - twist, -1 + twist, 0);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(actual(0) < 1e-2);
    EXPECT(actual(1) > M_PI - 1e-2)
  }
  {
    Unit3 p(0, 1, 0), q(0 + twist, -1 - twist, 0);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(actual(0) < 1e-2);
    EXPECT(actual(1) < -M_PI + 1e-2)
  }
}

//*******************************************************************************
// Wrapper to make basis return a Vector6 so we can test numerical derivatives.
Vector6 BasisTest(const Unit3& p, OptionalJacobian<6, 2> H) {
  Matrix32 B = p.basis(H);
  Vector6 B_vec;
  B_vec << B.col(0), B.col(1);
  return B_vec;
}

TEST(Unit3, basis) {
  Unit3 p(0.1, -0.2, 0.9);

  Matrix expected(3, 2);
  expected << 0.0, -0.994169047, 0.97618706, -0.0233922129, 0.216930458, 0.105264958;

  Matrix62 actualH;
  Matrix62 expectedH = numericalDerivative11<Vector6, Unit3>(
      std::bind(BasisTest, std::placeholders::_1, boost::none), p);

  // without H, first time
  EXPECT(assert_equal(expected, p.basis(), 1e-6));

  // without H, cached
  EXPECT(assert_equal(expected, p.basis(), 1e-6));

  // with H, first time
  EXPECT(assert_equal(expected, p.basis(actualH), 1e-6));
  EXPECT(assert_equal(expectedH, actualH, 1e-5));

  // with H, cached
  EXPECT(assert_equal(expected, p.basis(actualH), 1e-6));
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

//*******************************************************************************
/// Check the basis derivatives of a bunch of random Unit3s.
TEST(Unit3, basis_derivatives) {
  int num_tests = 100;
  std::mt19937 rng(42);
  for (int i = 0; i < num_tests; i++) {
    Unit3 p = Unit3::Random(rng);

    Matrix62 actualH;
    p.basis(actualH);

    Matrix62 expectedH = numericalDerivative11<Vector6, Unit3>(
        std::bind(BasisTest, std::placeholders::_1, boost::none), p);
    EXPECT(assert_equal(expectedH, actualH, 1e-5));
  }
}

//*******************************************************************************
TEST(Unit3, retract) {
  {
    Unit3 p;
    Vector2 v(0.5, 0);
    Unit3 expected(0.877583, 0, 0.479426);
    Unit3 actual = p.retract(v);
    EXPECT(assert_equal(expected, actual, 1e-6));
    EXPECT(assert_equal(v, p.localCoordinates(actual), 1e-5));
  }
  {
    Unit3 p;
    Vector2 v(0, 0);
    Unit3 actual = p.retract(v);
    EXPECT(assert_equal(p, actual, 1e-6));
    EXPECT(assert_equal(v, p.localCoordinates(actual), 1e-5));
  }
}

//*******************************************************************************
TEST (Unit3, jacobian_retract) {
  Matrix22 H;
  Unit3 p;
  std::function<Unit3(const Vector2&)> f =
      std::bind(&Unit3::retract, p, std::placeholders::_1, boost::none);
  {
      Vector2 v (-0.2, 0.1);
      p.retract(v, H);
      Matrix H_expected_numerical = numericalDerivative11(f, v);
      EXPECT(assert_equal(H_expected_numerical, H, 1e-5));
  }
  {
      Vector2 v (0, 0);
      p.retract(v, H);
      Matrix H_expected_numerical = numericalDerivative11(f, v);
      EXPECT(assert_equal(H_expected_numerical, H, 1e-5));
  }
}

//*******************************************************************************
TEST(Unit3, retract_expmap) {
  Unit3 p;
  Vector2 v((M_PI / 2.0), 0);
  Unit3 expected(Point3(0, 0, 1));
  Unit3 actual = p.retract(v);
  EXPECT(assert_equal(expected, actual, 1e-5));
  EXPECT(assert_equal(v, p.localCoordinates(actual), 1e-5));
}

//*******************************************************************************
TEST(Unit3, Random) {
  std::mt19937 rng(42);
  // Check that means are all zero at least
  Point3 expectedMean(0,0,0), actualMean(0,0,0);
  for (size_t i = 0; i < 100; i++)
    actualMean = actualMean + Unit3::Random(rng).point3();
  actualMean = actualMean / 100;
  EXPECT(assert_equal(expectedMean,actualMean,0.1));
}

//*******************************************************************************
// New test that uses Unit3::Random
TEST(Unit3, localCoordinates_retract) {
  std::mt19937 rng(42);
  size_t numIterations = 10000;

  for (size_t i = 0; i < numIterations; i++) {
    // Create two random Unit3s
    const Unit3 s1 = Unit3::Random(rng);
    const Unit3 s2 = Unit3::Random(rng);
    // Check that they are not at opposite ends of the sphere, which is ill defined
    if (s1.unitVector().dot(s2.unitVector())<-0.9) continue;

    // Check if the local coordinates and retract return consistent results.
    Vector v12 = s1.localCoordinates(s2);
    Unit3 actual_s2 = s1.retract(v12);
    EXPECT(assert_equal(s2, actual_s2, 1e-5));
  }
}

//*************************************************************************
TEST (Unit3, FromPoint3) {
  Matrix actualH;
  Point3 point(1, -2, 3); // arbitrary point
  Unit3 expected(point);
  EXPECT(assert_equal(expected, Unit3::FromPoint3(point, actualH), 1e-5));
  Matrix expectedH = numericalDerivative11<Unit3, Point3>(
      std::bind(Unit3::FromPoint3, std::placeholders::_1, boost::none), point);
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

//*******************************************************************************
TEST(Unit3, ErrorBetweenFactor) {
  std::vector<Unit3> data;
  data.push_back(Unit3(1.0, 0.0, 0.0));
  data.push_back(Unit3(0.0, 0.0, 1.0));

  NonlinearFactorGraph graph;
  Values initial_values;

  // Add prior factors.
  SharedNoiseModel R_prior = noiseModel::Unit::Create(2);
  for (size_t i = 0; i < data.size(); i++) {
    graph.addPrior(U(i), data[i], R_prior);
  }

  // Add process factors using the dot product error function.
  SharedNoiseModel R_process = noiseModel::Isotropic::Sigma(2, 0.01);
  for (size_t i = 0; i < data.size() - 1; i++) {
    Expression<Vector2> exp(Expression<Unit3>(U(i)), &Unit3::errorVector,
                            Expression<Unit3>(U(i + 1)));
    graph.addExpressionFactor<Vector2>(R_process, Vector2::Zero(), exp);
  }

  // Add initial values. Since there is no identity, just pick something.
  for (size_t i = 0; i < data.size(); i++) {
    initial_values.insert(U(i), Unit3(0.0, 1.0, 0.0));
  }

  Values values = GaussNewtonOptimizer(graph, initial_values).optimize();

  // Check that the y-value is very small for each.
  for (size_t i = 0; i < data.size(); i++) {
    EXPECT(assert_equal(0.0, values.at<Unit3>(U(i)).unitVector().y(), 1e-3));
  }

  // Check that the dot product between variables is close to 1.
  for (size_t i = 0; i < data.size() - 1; i++) {
    EXPECT(assert_equal(1.0, values.at<Unit3>(U(i)).dot(values.at<Unit3>(U(i + 1))), 1e-2));
  }
}

TEST(Unit3, CopyAssign) {
  Unit3 p{1, 0.2, 0.3};

  EXPECT(p.error(p).isZero());

  p = Unit3{-1, 2, 8};
  EXPECT(p.error(p).isZero());
}

/* ************************************************************************* */
TEST(actualH, Serialization) {
  Unit3 p(0, 1, 0);
  EXPECT(serializationTestHelpers::equalsObj(p));
  EXPECT(serializationTestHelpers::equalsXML(p));
  EXPECT(serializationTestHelpers::equalsBinary(p));
}


/* ************************************************************************* */
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
