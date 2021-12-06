/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SphericalCamera.h
 * @brief Calibrated camera with spherical projection
 * @date Aug 26, 2021
 * @author Luca Carlone
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/SphericalCamera.h>

#include <cmath>
#include <iostream>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

typedef SphericalCamera Camera;

// static const Cal3_S2 K(625, 625, 0, 0, 0);
//
static const Pose3 pose(Rot3(Vector3(1, -1, -1).asDiagonal()),
                        Point3(0, 0, 0.5));
static const Camera camera(pose);
//
static const Pose3 pose1(Rot3(), Point3(0, 1, 0.5));
static const Camera camera1(pose1);

static const Point3 point1(-0.08, -0.08, 0.0);
static const Point3 point2(-0.08, 0.08, 0.0);
static const Point3 point3(0.08, 0.08, 0.0);
static const Point3 point4(0.08, -0.08, 0.0);

// manually computed in matlab
static const Unit3 bearing1(-0.156054862928174, 0.156054862928174,
                            0.975342893301088);
static const Unit3 bearing2(-0.156054862928174, -0.156054862928174,
                            0.975342893301088);
static const Unit3 bearing3(0.156054862928174, -0.156054862928174,
                            0.975342893301088);
static const Unit3 bearing4(0.156054862928174, 0.156054862928174,
                            0.975342893301088);

static double depth = 0.512640224719052;
/* ************************************************************************* */
TEST(SphericalCamera, constructor) {
  EXPECT(assert_equal(pose, camera.pose()));
}

/* ************************************************************************* */
TEST(SphericalCamera, project) {
  // expected from manual calculation in Matlab
  EXPECT(assert_equal(camera.project(point1), bearing1));
  EXPECT(assert_equal(camera.project(point2), bearing2));
  EXPECT(assert_equal(camera.project(point3), bearing3));
  EXPECT(assert_equal(camera.project(point4), bearing4));
}

/* ************************************************************************* */
TEST(SphericalCamera, backproject) {
  EXPECT(assert_equal(camera.backproject(bearing1, depth), point1));
  EXPECT(assert_equal(camera.backproject(bearing2, depth), point2));
  EXPECT(assert_equal(camera.backproject(bearing3, depth), point3));
  EXPECT(assert_equal(camera.backproject(bearing4, depth), point4));
}

/* ************************************************************************* */
TEST(SphericalCamera, backproject2) {
  Point3 origin(0, 0, 0);
  Rot3 rot(1., 0., 0., 0., 0., 1., 0., -1., 0.);  // a camera1 looking down
  Camera camera(Pose3(rot, origin));

  Point3 actual = camera.backproject(Unit3(0, 0, 1), 1.);
  Point3 expected(0., 1., 0.);
  pair<Unit3, bool> x = camera.projectSafe(expected);

  EXPECT(assert_equal(expected, actual));
  EXPECT(assert_equal(Unit3(0, 0, 1), x.first));
  EXPECT(x.second);
}

/* ************************************************************************* */
static Unit3 project3(const Pose3& pose, const Point3& point) {
  return Camera(pose).project(point);
}

/* ************************************************************************* */
TEST(SphericalCamera, Dproject) {
  Matrix Dpose, Dpoint;
  Unit3 result = camera.project(point1, Dpose, Dpoint);
  Matrix numerical_pose = numericalDerivative21(project3, pose, point1);
  Matrix numerical_point = numericalDerivative22(project3, pose, point1);
  EXPECT(assert_equal(bearing1, result));
  EXPECT(assert_equal(numerical_pose, Dpose, 1e-7));
  EXPECT(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
static Vector2 reprojectionError2(const Pose3& pose, const Point3& point,
                                  const Unit3& measured) {
  return Camera(pose).reprojectionError(point, measured);
}

/* ************************************************************************* */
TEST(SphericalCamera, reprojectionError) {
  Matrix Dpose, Dpoint;
  Vector2 result = camera.reprojectionError(point1, bearing1, Dpose, Dpoint);
  Matrix numerical_pose =
      numericalDerivative31(reprojectionError2, pose, point1, bearing1);
  Matrix numerical_point =
      numericalDerivative32(reprojectionError2, pose, point1, bearing1);
  EXPECT(assert_equal(Vector2(0.0, 0.0), result));
  EXPECT(assert_equal(numerical_pose, Dpose, 1e-7));
  EXPECT(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
TEST(SphericalCamera, reprojectionError_noisy) {
  Matrix Dpose, Dpoint;
  Unit3 bearing_noisy = bearing1.retract(Vector2(0.01, 0.05));
  Vector2 result =
      camera.reprojectionError(point1, bearing_noisy, Dpose, Dpoint);
  Matrix numerical_pose =
      numericalDerivative31(reprojectionError2, pose, point1, bearing_noisy);
  Matrix numerical_point =
      numericalDerivative32(reprojectionError2, pose, point1, bearing_noisy);
  EXPECT(assert_equal(Vector2(-0.050282, 0.00833482), result, 1e-5));
  EXPECT(assert_equal(numerical_pose, Dpose, 1e-7));
  EXPECT(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
// Add a test with more arbitrary rotation
TEST(SphericalCamera, Dproject2) {
  static const Pose3 pose1(Rot3::Ypr(0.1, -0.1, 0.4), Point3(0, 0, -10));
  static const Camera camera(pose1);
  Matrix Dpose, Dpoint;
  camera.project2(point1, Dpose, Dpoint);
  Matrix numerical_pose = numericalDerivative21(project3, pose1, point1);
  Matrix numerical_point = numericalDerivative22(project3, pose1, point1);
  CHECK(assert_equal(numerical_pose, Dpose, 1e-7));
  CHECK(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
