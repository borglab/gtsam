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

#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/assign/std/vector.hpp>
#include <cmath>

using namespace boost::assign;
using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(Unit3)
GTSAM_CONCEPT_MANIFOLD_INST(Unit3)

//*******************************************************************************
Point3 point3_(const Unit3& p) {
  return p.point3();
}

TEST(Unit3, point3) {
  vector<Point3> ps;
  ps += Point3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1), Point3(1, 1, 0)
      / sqrt(2.0);
  Matrix actualH, expectedH;
  BOOST_FOREACH(Point3 p,ps) {
    Unit3 s(p);
    expectedH = numericalDerivative11<Point3, Unit3>(point3_, s);
    EXPECT(assert_equal(p, s.point3(actualH), 1e-8));
    EXPECT(assert_equal(expectedH, actualH, 1e-9));
  }
}

//*******************************************************************************
static Unit3 rotate_(const Rot3& R, const Unit3& p) {
  return R * p;
}

TEST(Unit3, rotate) {
  Rot3 R = Rot3::yaw(0.5);
  Unit3 p(1, 0, 0);
  Unit3 expected = Unit3(R.column(1));
  Unit3 actual = R * p;
  EXPECT(assert_equal(expected, actual, 1e-8));
  Matrix actualH, expectedH;

  {
    expectedH = numericalDerivative21(rotate_, R, p);
    R.rotate(p, actualH, boost::none);
    EXPECT(assert_equal(expectedH, actualH, 1e-9));
  }
  {
    expectedH = numericalDerivative22(rotate_, R, p);
    R.rotate(p, boost::none, actualH);
    EXPECT(assert_equal(expectedH, actualH, 1e-9));
  }
}

//*******************************************************************************
static Unit3 unrotate_(const Rot3& R, const Unit3& p) {
  return R.unrotate(p);
}

TEST(Unit3, unrotate) {
  Rot3 R = Rot3::yaw(-M_PI / 4.0);
  Unit3 p(1, 0, 0);
  Unit3 expected = Unit3(1, 1, 0);
  Unit3 actual = R.unrotate(p);
  EXPECT(assert_equal(expected, actual, 1e-8));

  Matrix actualH, expectedH;
  {
    expectedH = numericalDerivative21(unrotate_, R, p);
    R.unrotate(p, actualH, boost::none);
    EXPECT(assert_equal(expectedH, actualH, 1e-9));
  }
  {
    expectedH = numericalDerivative22(unrotate_, R, p);
    R.unrotate(p, boost::none, actualH);
    EXPECT(assert_equal(expectedH, actualH, 1e-9));
  }
}

//*******************************************************************************
TEST(Unit3, error) {
  Unit3 p(1, 0, 0), q = p.retract(Vector2(0.5, 0)), //
  r = p.retract(Vector2(0.8, 0));
  EXPECT(assert_equal((Vector)(Vector2(0, 0)), p.error(p), 1e-8));
  EXPECT(assert_equal((Vector)(Vector2(0.479426, 0)), p.error(q), 1e-5));
  EXPECT(assert_equal((Vector)(Vector2(0.717356, 0)), p.error(r), 1e-5));

  Matrix actual, expected;
  {
    expected = numericalDerivative11<Vector2,Unit3>(
        boost::bind(&Unit3::error, &p, _1, boost::none), q);
    p.error(q, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-9));
  }
  {
    expected = numericalDerivative11<Vector2,Unit3>(
        boost::bind(&Unit3::error, &p, _1, boost::none), r);
    p.error(r, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-9));
  }
}

//*******************************************************************************
TEST(Unit3, distance) {
  Unit3 p(1, 0, 0), q = p.retract(Vector2(0.5, 0)), //
  r = p.retract(Vector2(0.8, 0));
  EXPECT_DOUBLES_EQUAL(0, p.distance(p), 1e-8);
  EXPECT_DOUBLES_EQUAL(0.47942553860420301, p.distance(q), 1e-8);
  EXPECT_DOUBLES_EQUAL(0.71735609089952279, p.distance(r), 1e-8);

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalGradient<Unit3>(
        boost::bind(&Unit3::distance, &p, _1, boost::none), q);
    p.distance(q, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-9));
  }
  {
    expected = numericalGradient<Unit3>(
        boost::bind(&Unit3::distance, &p, _1, boost::none), r);
    p.distance(r, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-9));
  }
}

//*******************************************************************************

TEST(Unit3, localCoordinates) {
  {
    Unit3 p, q;
    Vector2 expected = Vector2::Zero();
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(zero(2), actual, 1e-8));
    EXPECT(assert_equal(q, p.retract(expected), 1e-8));
  }
  {
    Unit3 p, q(1, 6.12385e-21, 0);
    Vector2 expected = Vector2::Zero();
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(zero(2), actual, 1e-8));
    EXPECT(assert_equal(q, p.retract(expected), 1e-8));
  }
  {
    Unit3 p, q(-1, 0, 0);
    Vector2 expected(M_PI, 0);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(expected, actual, 1e-8));
    EXPECT(assert_equal(q, p.retract(expected), 1e-8));
  }
  {
    Unit3 p, q(0, 1, 0);
    Vector2 expected(0,-M_PI_2);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(expected, actual, 1e-8));
    EXPECT(assert_equal(q, p.retract(expected), 1e-8));
  }
  {
    Unit3 p, q(0, -1, 0);
    Vector2 expected(0, M_PI_2);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(expected, actual, 1e-8));
    EXPECT(assert_equal(q, p.retract(expected), 1e-8));
  }
  {
    Unit3 p(0,1,0), q(0,-1,0);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(q, p.retract(actual), 1e-8));
  }
  {
    Unit3 p(0,0,1), q(0,0,-1);
    Vector2 actual = p.localCoordinates(q);
    EXPECT(assert_equal(q, p.retract(actual), 1e-8));
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
TEST(Unit3, basis) {
  Unit3 p;
  Matrix32 expected;
  expected << 0, 0, 0, -1, 1, 0;
  Matrix actual = p.basis();
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*******************************************************************************
TEST(Unit3, retract) {
  {
    Unit3 p;
    Vector2 v(0.5, 0);
    Unit3 expected(0.877583, 0, 0.479426);
    Unit3 actual = p.retract(v);
    EXPECT(assert_equal(expected, actual, 1e-6));
    EXPECT(assert_equal(v, p.localCoordinates(actual), 1e-8));
  }
  {
    Unit3 p;
    Vector2 v(0, 0);
    Unit3 actual = p.retract(v);
    EXPECT(assert_equal(p, actual, 1e-6));
    EXPECT(assert_equal(v, p.localCoordinates(actual), 1e-8));
  }
}

//*******************************************************************************
TEST(Unit3, retract_expmap) {
  Unit3 p;
  Vector2 v((M_PI / 2.0), 0);
  Unit3 expected(Point3(0, 0, 1));
  Unit3 actual = p.retract(v);
  EXPECT(assert_equal(expected, actual, 1e-8));
  EXPECT(assert_equal(v, p.localCoordinates(actual), 1e-8));
}

//*******************************************************************************
TEST(Unit3, Random) {
  boost::mt19937 rng(42);
  // Check that means are all zero at least
  Point3 expectedMean, actualMean;
  for (size_t i = 0; i < 100; i++)
    actualMean = actualMean + Unit3::Random(rng).point3();
  actualMean = actualMean / 100;
  EXPECT(assert_equal(expectedMean,actualMean,0.1));
}

//*******************************************************************************
// New test that uses Unit3::Random
TEST(Unit3, localCoordinates_retract) {
  boost::mt19937 rng(42);
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
    EXPECT(assert_equal(s2, actual_s2, 1e-9));
  }
}

//*************************************************************************
TEST (Unit3, FromPoint3) {
  Matrix actualH;
  Point3 point(1, -2, 3); // arbitrary point
  Unit3 expected(point);
  EXPECT(assert_equal(expected, Unit3::FromPoint3(point, actualH), 1e-8));
  Matrix expectedH = numericalDerivative11<Unit3, Point3>(
      boost::bind(Unit3::FromPoint3, _1, boost::none), point);
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
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
  srand(time(NULL));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
