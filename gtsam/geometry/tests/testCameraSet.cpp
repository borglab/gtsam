/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testCameraSet.cpp
 *  @brief  Unit tests for testCameraSet Class
 *  @author Frank Dellaert
 *  @date   Feb 19, 2015
 */

#include <gtsam/geometry/CameraSet.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Cal3Bundler test
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
TEST(CameraSet, Pinhole) {
  typedef PinholeCamera<Cal3Bundler> Camera;
  typedef vector<Point2> ZZ;
  CameraSet<Camera> set;
  Camera camera;
  set.push_back(camera);
  set.push_back(camera);
  Point3 p(0, 0, 1);
  CHECK(assert_equal(set, set));
  CameraSet<Camera> set2 = set;
  set2.push_back(camera);
  CHECK(!set.equals(set2));

  // Check measurements
  Point2 expected;
  ZZ z = set.project(p);
  CHECK(assert_equal(expected, z[0]));
  CHECK(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix46 actualF;
  Matrix43 actualE;
  Matrix43 actualH;
  {
    Matrix26 F1;
    Matrix23 E1;
    Matrix23 H1;
    camera.project(p, F1, E1, H1);
    actualE << E1, E1;
    actualF << F1, F1;
    actualH << H1, H1;
  }

  // Check computed derivatives
  Matrix F, E, H;
  set.project(p, F, E, H);
  CHECK(assert_equal(actualF, F));
  CHECK(assert_equal(actualE, E));
  CHECK(assert_equal(actualH, H));
}

/* ************************************************************************* */
#include <gtsam/geometry/StereoCamera.h>
TEST(CameraSet, Stereo) {
  typedef vector<StereoPoint2> ZZ;
  CameraSet<StereoCamera> set;
  StereoCamera camera;
  set.push_back(camera);
  set.push_back(camera);
  Point3 p(0, 0, 1);
  EXPECT_LONGS_EQUAL(6, traits<StereoCamera>::dimension);

  // Check measurements
  StereoPoint2 expected(0, -1, 0);
  ZZ z = set.project(p);
  CHECK(assert_equal(expected, z[0]));
  CHECK(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix66 actualF;
  Matrix63 actualE;
  {
    Matrix36 F1;
    Matrix33 E1;
    camera.project(p, F1, E1);
    actualE << E1, E1;
    actualF << F1, F1;
  }

  // Check computed derivatives
  Matrix F, E;
  set.project(p, F, E);
  CHECK(assert_equal(actualF, F));
  CHECK(assert_equal(actualE, E));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

