/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testOrientedPlane3.cpp
 * @date Dec 19, 2012
 * @author Alex Trevor
 * @author Zhaoyang Lv
 * @brief Tests the OrientedPlane3 class
 */

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace std::placeholders;
using namespace gtsam;
using namespace std;
using boost::none;

GTSAM_CONCEPT_TESTABLE_INST(OrientedPlane3)
GTSAM_CONCEPT_MANIFOLD_INST(OrientedPlane3)

//*******************************************************************************
TEST(OrientedPlane3, getMethods) {
  Vector4 c;
  c << -1, 0, 0, 5;
  OrientedPlane3 plane1(c);
  OrientedPlane3 plane2(c[0], c[1], c[2], c[3]);
  Vector4 coefficient1 = plane1.planeCoefficients();
  double distance1 = plane1.distance();
  EXPECT(assert_equal(coefficient1, c, 1e-8));
  EXPECT(assert_equal(Unit3(-1,0,0).unitVector(), plane1.normal().unitVector()));
  EXPECT_DOUBLES_EQUAL(distance1, 5, 1e-8);
  Vector4 coefficient2 = plane2.planeCoefficients();
  double distance2 = plane2.distance();
  EXPECT(assert_equal(coefficient2, c, 1e-8));
  EXPECT_DOUBLES_EQUAL(distance2, 5, 1e-8);
  EXPECT(assert_equal(Unit3(-1,0,0).unitVector(), plane2.normal().unitVector()));
}


//*******************************************************************************
OrientedPlane3 transform_(const OrientedPlane3& plane,  const Pose3& xr) {
  return plane.transform(xr);
}

TEST(OrientedPlane3, transform) {
  gtsam::Pose3 pose(gtsam::Rot3::Ypr(-M_PI / 4.0, 0.0, 0.0),
                    gtsam::Point3(2.0, 3.0, 4.0));
  OrientedPlane3 plane(-1, 0, 0, 5);
  OrientedPlane3 expectedPlane(-sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0, 0.0, 3);
  OrientedPlane3 transformedPlane = plane.transform(pose, none, none);
  EXPECT(assert_equal(expectedPlane, transformedPlane, 1e-5));

  // Test the jacobians of transform
  Matrix actualH1, expectedH1, actualH2, expectedH2;
  expectedH1 = numericalDerivative21(transform_, plane, pose);
  plane.transform(pose, actualH1, none);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-5));

  expectedH2 = numericalDerivative22(transform_, plane, pose);
  plane.transform(pose, none, actualH2);
  EXPECT(assert_equal(expectedH2, actualH2, 1e-5));
}

//*******************************************************************************
// Returns a any size random vector
inline static Vector randomVector(const Vector& minLimits,
    const Vector& maxLimits) {

  // Get the number of dimensions and create the return vector
  size_t numDims = minLimits.size();
  Vector vector = Vector::Zero(numDims);

  // Create the random vector
  for (size_t i = 0; i < numDims; i++) {
    double range = maxLimits(i) - minLimits(i);
    vector(i) = (((double) rand()) / RAND_MAX) * range + minLimits(i);
  }
  return vector;
}

//*******************************************************************************
TEST(OrientedPlane3, localCoordinates_retract) {
  size_t numIterations = 10000;
  Vector4 minPlaneLimit, maxPlaneLimit;
  minPlaneLimit << -1.0, -1.0, -1.0, 0.01;
  maxPlaneLimit << 1.0, 1.0, 1.0, 10.0;

  Vector3 minXiLimit, maxXiLimit;
  minXiLimit << -M_PI, -M_PI, -10.0;
  maxXiLimit << M_PI, M_PI, 10.0;
  for (size_t i = 0; i < numIterations; i++) {
    // Create a Plane
    OrientedPlane3 p1(randomVector(minPlaneLimit, maxPlaneLimit));
    Vector v12 = randomVector(minXiLimit, maxXiLimit);

    // Magnitude of the rotation can be at most pi
    if (v12.head<3>().norm() > M_PI)
      v12.head<3>() = v12.head<3>() / M_PI;
    OrientedPlane3 p2 = p1.retract(v12);

    // Check if the local coordinates and retract return the same results.
    Vector actual_v12 = p1.localCoordinates(p2);
    EXPECT(assert_equal(v12, actual_v12, 1e-6));
    OrientedPlane3 actual_p2 = p1.retract(actual_v12);
    EXPECT(assert_equal(p2, actual_p2, 1e-6));
  }
}

//*******************************************************************************
TEST(OrientedPlane3, errorVector) {
  OrientedPlane3 plane1(-1, 0.1, 0.2, 5);
  OrientedPlane3 plane2(-1.1, 0.2, 0.3, 5.4);

  // Hard-coded regression values, to ensure the result doesn't change.
  EXPECT(assert_equal((Vector) Z_3x1, plane1.errorVector(plane1), 1e-8));
  EXPECT(assert_equal(Vector3(-0.0677674148, -0.0760543588, -0.4),
                      plane1.errorVector(plane2), 1e-5));

  // Test the jacobians of transform
  Matrix33 actualH1, expectedH1, actualH2, expectedH2;

  Vector3 actual = plane1.errorVector(plane2, actualH1, actualH2);
  EXPECT(assert_equal(plane1.normal().errorVector(plane2.normal()),
                      Vector2(actual[0], actual[1])));
  EXPECT(assert_equal(plane1.distance() - plane2.distance(), actual[2]));

  std::function<Vector3(const OrientedPlane3&, const OrientedPlane3&)> f =
      std::bind(&OrientedPlane3::errorVector, std::placeholders::_1,
                std::placeholders::_2, boost::none, boost::none);
  expectedH1 = numericalDerivative21(f, plane1, plane2);
  expectedH2 = numericalDerivative22(f, plane1, plane2);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-5));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-5));
}

//*******************************************************************************
TEST(OrientedPlane3, jacobian_retract) {
  OrientedPlane3 plane(-1, 0.1, 0.2, 5);
  Matrix33 H_actual;
  std::function<OrientedPlane3(const Vector3&)> f = std::bind(
      &OrientedPlane3::retract, plane, std::placeholders::_1, boost::none);
  {
      Vector3 v(-0.1, 0.2, 0.3);
      plane.retract(v, H_actual);
      Matrix H_expected_numerical = numericalDerivative11(f, v);
      EXPECT(assert_equal(H_expected_numerical, H_actual, 1e-5));
  }
  {
      Vector3 v(0, 0, 0);
      plane.retract(v, H_actual);
      Matrix H_expected_numerical = numericalDerivative11(f, v);
      EXPECT(assert_equal(H_expected_numerical, H_actual, 1e-5));
  }
}

//*******************************************************************************
TEST(OrientedPlane3, getMethodJacobians) {
  OrientedPlane3 plane(-1, 0.1, 0.2, 5);
  Matrix33 H_retract, H_getters;
  Matrix23 H_normal;
  Matrix13 H_distance;

  // The getter's jacobians lie exactly on the tangent space
  // so they should exactly equal the retract jacobian for the zero vector.
  Vector3 v(0, 0, 0);
  plane.retract(v, H_retract);
  plane.normal(H_normal);
  plane.distance(H_distance);
  H_getters << H_normal, H_distance;
  EXPECT(assert_equal(H_retract, H_getters, 1e-5));
}

/* ************************************************************************* */
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
