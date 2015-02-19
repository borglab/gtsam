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
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
TEST(CameraSet, Pinhole) {
  typedef PinholeCamera<Cal3Bundler> Camera;
  typedef vector<Point2> ZZ;
  CameraSet<Camera> set;
  Camera camera;
  set.add(camera);
  set.add(camera);
  Point3 p(0, 0, 1);
  // Calculate actual
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
  Point2 expected;
  Matrix F, E, H;
  ZZ z = set.project(p, F, E, H);
  CHECK(assert_equal(expected, z[0]));
  CHECK(assert_equal(expected, z[1]));
  CHECK(assert_equal(actualF, F));
  CHECK(assert_equal(actualE, E));
  CHECK(assert_equal(actualH, H));
}

/* ************************************************************************* */
#include <gtsam/geometry/StereoCamera.h>
TEST(CameraSet, Stereo) {
  CameraSet<StereoCamera> f;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

