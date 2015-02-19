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

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
TEST(CameraSet, Pinhole) {
  typedef PinholeCamera<Cal3Bundler> Camera;
  CameraSet<Camera> set;
  set.add(Camera());
  set.add(Camera());
  Point3 p(0,0,1);
  Matrix F,E,H;
  vector<Point2> z = set.project(p,F,E,H);
  Point2 expected;
  CHECK(assert_equal(expected,z[0]));
  CHECK(assert_equal(expected,z[1]));
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

