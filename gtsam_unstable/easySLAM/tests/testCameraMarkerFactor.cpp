/**********************************************************
Written by Alireza Fathi, 17th of Nov 2008
**********************************************************/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/NonlinearFactorGraph.h>
#include <gtsam/VSLAMFactor.h>
#include <gtsam/numericalDerivative.h>

#include "CameraMarkerFactor.h"
#include "Marker.h"
#include "easyExample.h"
#include "homography.h"

using namespace std;
using namespace gtsam;

Cal3_S2 K(625, 625, 0, 0, 0);
double markerSize = 160.0;

/* ************************************************************************* */
// make sure that homography and hGetP functions are the complement of each
// other.
TEST(cameraMarkerFactor, problem1) {
  double markerSize = 160.0;
  Cal3_S2 K(687.48319, 690.83434, 0.0, 333.41606, 242.98570);

  // Vector z(8); z(0) = 181.683; z(1) = 248.006; z(2) = 131.442; z(3) =
  // 246.257; z(4) = 131.651; z(5) = 196.97; z(6) = 181.84; z(7) = 196.707;
  Vector z(8);
  z(0) = 531.597;
  z(1) = 304.878;
  z(2) = 509.092;
  z(3) = 298.649;
  z(4) = 513.456;
  z(5) = 237.47;
  z(6) = 537.035;
  z(7) = 237.55;

  ARRobotMarker am(0, 18, 531.597, 304.878, 509.092, 298.649, 513.456, 237.47,
                   537.035, 237.55, markerSize, K);
  // ARRobotMarker
  // am(0,18,181.683,248.006,131.442,246.257,131.651,196.97,181.84,196.707);
  Pose3 marker = am.getEstimatedPose();
  Pose3 pose;  // origin
  // Pose3 pose(inverse(p.matrix()));

  SimpleCamera camera(K, pose);
  Marker markerObj(marker, markerSize);
  // Vector hAll = hGetAll(camera, marker, markerSize);
  Vector hAll = hGetAll(camera, markerObj);
  CHECK(assert_equal(z, hAll, 20));  // TODO: 20 is very high !
}
/* ************************************************************************* */
// make sure that homography and hGetP functions are the complement of each
// other.
TEST(cameraMarkerFactor, problem2) {
  double markerSize = 160.0;
  Cal3_S2 K(687.48319, 690.83434, 0.0, 333.41606, 242.98570);

  // Vector z(8); z(0) = 181.683; z(1) = 248.006; z(2) = 131.442; z(3) =
  // 246.257; z(4) = 131.651; z(5) = 196.97; z(6) = 181.84; z(7) = 196.707;
  Vector z(8);
  z(0) = 531.597;
  z(1) = 304.878;
  z(2) = 509.092;
  z(3) = 298.649;
  z(4) = 513.456;
  z(5) = 237.47;
  z(6) = 537.035;
  z(7) = 237.55;

  ARRobotMarker am(0, 18, 531.597, 304.878, 509.092, 298.649, 513.456, 237.47,
                   537.035, 237.55, markerSize, K);
  // ARRobotMarker
  // am(0,18,181.683,248.006,131.442,246.257,131.651,196.97,181.84,196.707);
  Pose3 marker;  // origin
  Pose3 pose = am.getEstimatedPose().inverse();

  SimpleCamera camera(K, pose);
  Marker markerObj(marker, markerSize);
  // Vector hAll = hGetAll(camera, marker, markerSize);
  Vector hAll = hGetAll(camera, markerObj);
  CHECK(assert_equal(z, hAll, 100));  // TODO: 100 is very high !
}

/* ************************************************************************* */
TEST(CameraMarkerFactor, hGetPa2) {
  const Pose3 &x1 = camera1(), m1 = marker1();

  CHECK(assert_equal(get_marker_point(markerSize, 1), Point3(-80, -80, 0)));
  CHECK(assert_equal(get_marker_point(markerSize, 2), Point3(-80, 80, 0)));
  CHECK(assert_equal(get_marker_point(markerSize, 3), Point3(80, 80, 0)));
  CHECK(assert_equal(get_marker_point(markerSize, 4), Point3(80, -80, 0)));

  SimpleCamera camera(K, x1);
  /*
    CHECK(assert_equal( hGetP(camera, m1, markerSize, 1), Point2(-100, 100) ));
    CHECK(assert_equal( hGetP(camera, m1, markerSize, 2), Point2(-100,-100) ));
    CHECK(assert_equal( hGetP(camera, m1, markerSize, 3), Point2( 100,-100) ));
    CHECK(assert_equal( hGetP(camera, m1, markerSize, 4), Point2( 100, 100) ));
  */
  Marker markerObj(m1, markerSize);
  CHECK(assert_equal(hGetP(camera, markerObj, 1), Point2(-100, 100)));
  CHECK(assert_equal(hGetP(camera, markerObj, 2), Point2(-100, -100)));
  CHECK(assert_equal(hGetP(camera, markerObj, 3), Point2(100, -100)));
  CHECK(assert_equal(hGetP(camera, markerObj, 4), Point2(100, 100)));
}

/* ************************************************************************* */

// temp function for test
Point2 hGetPa(const Pose3& cameraPose, const Pose3& markerPose) {
  SimpleCamera camera(K, cameraPose);
  Marker markerObj(markerPose, markerSize);
  // return hGetP(camera, markerPose, markerSize, 1);
  return hGetP(camera, markerObj, 1);
}

TEST(CameraMarkerFactor, DhGetP_pose) {
  Pose3 cameraPose = camera1(), markerPose = marker1();
  SimpleCamera camera(K, cameraPose);
  Marker markerObj(markerPose, markerSize);
  // Matrix computed = DhGetP_pose(camera, markerPose, markerSize,1);
  Matrix computed = DhGetP_pose(camera, markerObj, 1);
  Matrix numerical = numericalDerivative21(hGetPa, cameraPose, markerPose);
  CHECK(assert_equal(computed, numerical, 1e-7));
}

TEST(CameraMarkerFactor, DhGetP_marker) {
  Pose3 cameraPose = camera1(), markerPose = marker1();
  SimpleCamera camera(K, cameraPose);
  Marker markerObj(markerPose, markerSize);
  // Matrix computed = DhGetP_marker(camera, markerPose, markerSize,1);
  Matrix computed = DhGetP_marker(camera, markerObj, 1);
  Matrix numerical = numericalDerivative22(hGetPa, cameraPose, markerPose);
  CHECK(assert_equal(computed, numerical, 1e-7));
}

/* ************************************************************************* */
TEST(CameraMarkerFactor, linearize) {
  // grab factor and linearize
  CameraMarkerFactor::shared_ptr factor = createExampleFactor1();
  FGConfig config = createExampleConfig();

  LinearFactor::shared_ptr actual = factor->linearize(config);

  // create linear factor with two 8*6 matrices and a 8-dim RHS
  // the combined linear factor
  double a = 8, B = 50, c = 0.625, D = 320.5, E = 320.5, z = 0.1, Z = 0;

  Matrix Amarker =
      Matrix_(8, 6, +a, -a, B, c, Z, -z, -a, +a, B, Z, -c, +z, -a, -a, -B, c, Z,
              -z, -a, -a, B, Z, -c, -z, +a, -a, -B, c, Z, +z, -a, +a, -B, Z, -c,
              -z, -a, -a, B, c, Z, +z, -a, -a, -B, Z, -c, +z);

  Matrix Apose =
      Matrix_(8, 6, -a, +E, -B, -c, Z, +z, +D, -a, -B, Z, c, -z, +a, +E, B, -c,
              Z, +z, +D, +a, -B, Z, c, +z, -a, +E, B, -c, Z, -z, +D, -a, B, Z,
              c, +z, +a, +E, -B, -c, Z, -z, +D, +a, B, Z, c, -z);

  // the RHS is near zero
  Vector rhs(8, 0.0);

  LinearFactor expected("x0", Apose, rhs);

  const double error = 1;  // only approximate
  CHECK(actual->equals(expected, error));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  TestRegistry::runAllTests(tr);
  return 0;
}
/* ************************************************************************* */
