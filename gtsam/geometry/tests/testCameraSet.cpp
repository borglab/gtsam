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
  EXPECT(assert_equal(set, set));
  CameraSet<Camera> set2 = set;
  set2.push_back(camera);
  EXPECT(!set.equals(set2));

  // Check measurements
  Point2 expected;
  ZZ z = set.project(p);
  EXPECT(assert_equal(expected, z[0]));
  EXPECT(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix43 actualE;
  Matrix43 actualH;
  Matrix F1;
  {
    Matrix23 E1;
    Matrix23 H1;
    camera.project(p, F1, E1, H1);
    actualE << E1, E1;
    actualH << H1, H1;
  }

  // Check computed derivatives
  CameraSet<Camera>::FBlocks F;
  Matrix E, H;
  set.project(p, F, E, H);
  LONGS_EQUAL(2,F.size());
  EXPECT(assert_equal(F1, F[0]));
  EXPECT(assert_equal(F1, F[1]));
  EXPECT(assert_equal(actualE, E));
  EXPECT(assert_equal(actualH, H));

  // Check errors
  ZZ measured;
  measured.push_back(Point2(1, 2));
  measured.push_back(Point2(3, 4));
  Vector4 expectedV;

  // reprojectionErrors
  expectedV << -1, -2, -3, -4;
  Vector actualV = set.reprojectionErrors(p, measured);
  EXPECT(assert_equal(expectedV, actualV));

  // reprojectionErrorsAtInfinity
  EXPECT(
      assert_equal(Point3(0, 0, 1),
          camera.backprojectPointAtInfinity(Point2())));
  actualV = set.reprojectionErrorsAtInfinity(p, measured);
  EXPECT(assert_equal(expectedV, actualV));
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
  EXPECT(assert_equal(expected, z[0]));
  EXPECT(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix63 actualE;
  Matrix F1;
  {
    Matrix33 E1;
    camera.project(p, F1, E1);
    actualE << E1, E1;
  }

  // Check computed derivatives
  CameraSet<StereoCamera>::FBlocks F;
  Matrix E;
  set.project(p, F, E);
  LONGS_EQUAL(2,F.size());
  EXPECT(assert_equal(F1, F[0]));
  EXPECT(assert_equal(F1, F[1]));
  EXPECT(assert_equal(actualE, E));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

