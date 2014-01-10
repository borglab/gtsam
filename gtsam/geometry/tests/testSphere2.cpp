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

#include <gtsam/geometry/Sphere2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(Sphere2)
GTSAM_CONCEPT_MANIFOLD_INST(Sphere2)

//*******************************************************************************
Point3 point3_(const Sphere2& p) {
  return p.point3();
}
TEST(Sphere2, point3) {
  vector<Point3> ps;
  ps += Point3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1), Point3(1, 1, 0)
      / sqrt(2);
  Matrix actualH, expectedH;
  BOOST_FOREACH(Point3 p,ps) {
    Sphere2 s(p);
    expectedH = numericalDerivative11<Point3, Sphere2>(point3_, s);
    EXPECT(assert_equal(p, s.point3(actualH), 1e-8));
    EXPECT(assert_equal(expectedH, actualH, 1e-9));
  }
}

//*******************************************************************************
static Sphere2 rotate_(const Rot3& R, const Sphere2& p) {
  return R * p;
}

TEST(Sphere2, rotate) {
  Rot3 R = Rot3::yaw(0.5);
  Sphere2 p(1, 0, 0);
  Sphere2 expected = Sphere2(R.column(1));
  Sphere2 actual = R * p;
  EXPECT(assert_equal(expected, actual, 1e-8));
  Matrix actualH, expectedH;
  // Use numerical derivatives to calculate the expected Jacobian
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
static Sphere2 unrotate_(const Rot3& R, const Sphere2& p) {
  return R.unrotate (p);
}

TEST(Sphere2, unrotate) {
  Rot3 R = Rot3::yaw(-M_PI/4.0);
  Sphere2 p(1, 0, 0);
  Sphere2 expected = Sphere2(1, 1, 0);
  Sphere2 actual = R.unrotate (p);
  EXPECT(assert_equal(expected, actual, 1e-8));
  Matrix actualH, expectedH;
  // Use numerical derivatives to calculate the expected Jacobian
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
TEST(Sphere2, error) {
  Sphere2 p(1, 0, 0), q = p.retract((Vector(2) << 0.5, 0), Sphere2::RENORM), //
  r = p.retract((Vector(2) << 0.8, 0), Sphere2::RENORM);
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
  Sphere2 p(1, 0, 0), q = p.retract((Vector(2) << 0.5, 0), Sphere2::RENORM), //
  r = p.retract((Vector(2) << 0.8, 0), Sphere2::RENORM);
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
  Sphere2 actual = p.retract(v, Sphere2::RENORM);
  EXPECT(assert_equal(expected, actual, 1e-8));
  EXPECT(assert_equal(v, p.localCoordinates(actual, Sphere2::RENORM), 1e-8));
}

//*******************************************************************************
TEST(Sphere2, retract_expmap) {
  Sphere2 p;
  Vector v(2);
  v << (M_PI/2.0), 0;
  Sphere2 expected(Point3(0, 0, 1));
  Sphere2 actual = p.retract(v, Sphere2::EXPMAP);
  EXPECT(assert_equal(expected, actual, 1e-8));
  EXPECT(assert_equal(v, p.localCoordinates(actual, Sphere2::EXPMAP), 1e-8));
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
// Let x and y be two Sphere2's.
// The equality x.localCoordinates(x.retract(v)) == v should hold.
TEST(Sphere2, localCoordinates_retract_expmap) {
  
  size_t numIterations = 10000;
  Vector minSphereLimit = Vector_(3, -1.0, -1.0, -1.0), maxSphereLimit =
      Vector_(3, 1.0, 1.0, 1.0);
  Vector minXiLimit = Vector_(2, -M_PI, -M_PI), maxXiLimit = Vector_(2, M_PI, M_PI);
  for (size_t i = 0; i < numIterations; i++) {

    // Sleep for the random number generator (TODO?: Better create all of them first).
    sleep(0);

    // Create the two Sphere2s.
    // Unlike the above case, we can use any two sphers.
    Sphere2 s1(Point3(randomVector(minSphereLimit, maxSphereLimit)));
//      Sphere2 s2 (Point3(randomVector(minSphereLimit, maxSphereLimit)));
    Vector v12 = randomVector(minXiLimit, maxXiLimit);
    
    // Magnitude of the rotation can be at most pi
    if (v12.norm () > M_PI)
      v12 = v12 / M_PI;
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

//*******************************************************************************
TEST(Sphere2, Random) {
  boost::random::mt19937 rng(42);
  // Check that is deterministic given same random seed
  Point3 expected(-0.667578, 0.671447, 0.321713);
  Point3 actual = Sphere2::Random(rng).point3();
  EXPECT(assert_equal(expected,actual,1e-5));
  // Check that means are all zero at least
  Point3 expectedMean, actualMean;
  for (size_t i = 0; i < 100; i++)
    actualMean = actualMean + Sphere2::Random(rng).point3();
  actualMean = actualMean/100;
  EXPECT(assert_equal(expectedMean,actualMean,0.1));
}

/* ************************************************************************* */
int main() {
  srand(time(NULL));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
