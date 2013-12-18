/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testSphere2.cpp
 * @date Feb 03, 2012
 * @author Can Erdogan
 * @brief Tests the Sphere2 class
 */

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Sphere2.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <boost/bind.hpp>

using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(Sphere2)
GTSAM_CONCEPT_MANIFOLD_INST(Sphere2)

//*******************************************************************************
TEST(Sphere2, rotate) {
  Rot3 R = Rot3::yaw(0.5);
  Sphere2 p(1, 0, 0);
  Sphere2 expected = Sphere2(R.column(0));
  Sphere2 actual = R * p;
  EXPECT(assert_equal(expected, actual, 1e-8));
  Matrix actualH, expectedH;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expectedH = numericalDerivative11<Sphere2, Rot3>(
        boost::bind(&Sphere2::Rotate, _1, p, boost::none, boost::none), R);
    Sphere2::Rotate(R, p, actualH, boost::none);
    EXPECT(assert_equal(expectedH, actualH, 1e-9));
  }
  {
    expectedH = numericalDerivative11<Sphere2, Sphere2>(
        boost::bind(&Sphere2::Rotate, R, _1, boost::none, boost::none), p);
    Sphere2::Rotate(R, p, boost::none, actualH);
    EXPECT(assert_equal(expectedH, actualH, 1e-9));
  }
}

//*******************************************************************************
TEST(Sphere2, error) {
  Sphere2 p(1, 0, 0), q = p.retract((Vector(2) << 0.5, 0)), //
  r = p.retract((Vector(2) << 0.8, 0));
  EXPECT(assert_equal((Vector(2) << 0, 0), p.error(p), 1e-8));
  EXPECT(assert_equal((Vector(2) << 0.447214, 0), p.error(q), 1e-5));
  EXPECT(assert_equal((Vector(2) << 0.624695, 0), p.error(r), 1e-5));

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalDerivative11<Sphere2>(
        boost::bind(&Sphere2::error, &p, _1, boost::none), q);
    p.error(q, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-9));
  }
  {
    expected = numericalDerivative11<Sphere2>(
        boost::bind(&Sphere2::error, &p, _1, boost::none), r);
    p.error(r, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-9));
  }
}

//*******************************************************************************
TEST(Sphere2, distance) {
  Sphere2 p(1, 0, 0), q = p.retract((Vector(2) << 0.5, 0)), //
  r = p.retract((Vector(2) << 0.8, 0));
  EXPECT_DOUBLES_EQUAL(0, p.distance(p), 1e-8);
  EXPECT_DOUBLES_EQUAL(0.44721359549995798, p.distance(q), 1e-8);
  EXPECT_DOUBLES_EQUAL(0.6246950475544244, p.distance(r), 1e-8);

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalGradient<Sphere2>(
        boost::bind(&Sphere2::distance, &p, _1, boost::none), q);
    p.distance(q, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-9));
  }
  {
    expected = numericalGradient<Sphere2>(
        boost::bind(&Sphere2::distance, &p, _1, boost::none), r);
    p.distance(r, actual);
    EXPECT(assert_equal(expected.transpose(), actual, 1e-9));
  }
}

//*******************************************************************************
TEST(Sphere2, localCoordinates0) {
  Sphere2 p;
  Vector expected = zero(2);
  Vector actual = p.localCoordinates(p);
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*******************************************************************************
TEST(Sphere2, basis) {
  Sphere2 p;
  Matrix expected(3, 2);
  expected << 0, 0, 0, -1, 1, 0;
  Matrix actual = p.basis();
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*******************************************************************************
TEST(Sphere2, retract) {
  Sphere2 p;
  Vector v(2);
  v << 0.5, 0;
  Sphere2 expected(Point3(1, 0, 0.5));
  Sphere2 actual = p.retract(v);
  EXPECT(assert_equal(expected, actual, 1e-8));
  EXPECT(assert_equal(v, p.localCoordinates(actual), 1e-8));
}

//*******************************************************************************
/// Returns a random vector
inline static Vector randomVector(const Vector& minLimits,
    const Vector& maxLimits) {

  // Get the number of dimensions and create the return vector
  size_t numDims = dim(minLimits);
  Vector vector = zero(numDims);

  // Create the random vector
  for (size_t i = 0; i < numDims; i++) {
    double range = maxLimits(i) - minLimits(i);
    vector(i) = (((double) rand()) / RAND_MAX) * range + minLimits(i);
  }
  return vector;
}

//*******************************************************************************
// Let x and y be two Sphere2's.
// The equality x.localCoordinates(x.retract(v)) == v should hold.
TEST(Sphere2, localCoordinates_retract) {

  size_t numIterations = 10000;
  Vector minSphereLimit = Vector_(3, -1.0, -1.0, -1.0), maxSphereLimit =
      Vector_(3, 1.0, 1.0, 1.0);
  Vector minXiLimit = Vector_(2, -1.0, -1.0), maxXiLimit = Vector_(2, 1.0, 1.0);
  for (size_t i = 0; i < numIterations; i++) {

    // Sleep for the random number generator (TODO?: Better create all of them first).
    sleep(0);

    // Create the two Sphere2s.
    // NOTE: You can not create two totally random Sphere2's because you cannot always compute
    // between two any Sphere2's. (For instance, they might be at the different sides of the circle).
    Sphere2 s1(Point3(randomVector(minSphereLimit, maxSphereLimit)));
//      Sphere2 s2 (Point3(randomVector(minSphereLimit, maxSphereLimit)));
    Vector v12 = randomVector(minXiLimit, maxXiLimit);
    Sphere2 s2 = s1.retract(v12);

    // Check if the local coordinates and retract return the same results.
    Vector actual_v12 = s1.localCoordinates(s2);
    EXPECT(assert_equal(v12, actual_v12, 1e-3));
    Sphere2 actual_s2 = s1.retract(actual_v12);
    EXPECT(assert_equal(s2, actual_s2, 1e-3));
  }
}

//*******************************************************************************
//TEST( Pose2, between )
//{
//  // <
//  //
//  //       ^
//  //
//  // *--0--*--*
//  Pose2 gT1(M_PI/2.0, Point2(1,2)); // robot at (1,2) looking towards y
//  Pose2 gT2(M_PI, Point2(-1,4));  // robot at (-1,4) loooking at negative x
//
//  Matrix actualH1,actualH2;
//  Pose2 expected(M_PI/2.0, Point2(2,2));
//  Pose2 actual1 = gT1.between(gT2);
//  Pose2 actual2 = gT1.between(gT2,actualH1,actualH2);
//  EXPECT(assert_equal(expected,actual1));
//  EXPECT(assert_equal(expected,actual2));
//
//  Matrix expectedH1 = Matrix_(3,3,
//      0.0,-1.0,-2.0,
//      1.0, 0.0,-2.0,
//      0.0, 0.0,-1.0
//  );
//  Matrix numericalH1 = numericalDerivative21<Pose2,Pose2,Pose2>(testing::between, gT1, gT2);
//  EXPECT(assert_equal(expectedH1,actualH1));
//  EXPECT(assert_equal(numericalH1,actualH1));
//  // Assert H1 = -AdjointMap(between(p2,p1)) as in doc/math.lyx
//  EXPECT(assert_equal(-gT2.between(gT1).AdjointMap(),actualH1));
//
//  Matrix expectedH2 = Matrix_(3,3,
//       1.0, 0.0, 0.0,
//       0.0, 1.0, 0.0,
//       0.0, 0.0, 1.0
//  );
//  Matrix numericalH2 = numericalDerivative22<Pose2,Pose2,Pose2>(testing::between, gT1, gT2);
//  EXPECT(assert_equal(expectedH2,actualH2));
//  EXPECT(assert_equal(numericalH2,actualH2));
//
//}

/* ************************************************************************* */
int main() {
  srand(time(NULL));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
