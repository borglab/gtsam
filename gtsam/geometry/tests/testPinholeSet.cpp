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

#include <gtsam/geometry/PinholeSet.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
#include <gtsam/geometry/CalibratedCamera.h>
TEST(PinholeSet, Stereo) {
  typedef vector<Point2> ZZ;
  PinholeSet<CalibratedCamera> set;
  CalibratedCamera camera;
  set.push_back(camera);
  set.push_back(camera);
  //  set.print("set: ");
  Point3 p(0, 0, 1);
  EXPECT_LONGS_EQUAL(6, traits<CalibratedCamera>::dimension);

  // Check measurements
  Point2 expected(0, 0);
  ZZ z = set.project2(p);
  EXPECT(assert_equal(expected, z[0]));
  EXPECT(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix43 actualE;
  Matrix F1;
  {
    Matrix23 E1;
    camera.project2(p, F1, E1);
    actualE << E1, E1;
  }

  // Check computed derivatives
  PinholeSet<CalibratedCamera>::FBlocks F;
  Matrix E;
  set.project2(p, F, E);
  LONGS_EQUAL(2, F.size());
  EXPECT(assert_equal(F1, F[0]));
  EXPECT(assert_equal(F1, F[1]));
  EXPECT(assert_equal(actualE, E));

  // Instantiate triangulateSafe
  // TODO triangulation does not work yet for CalibratedCamera
  // PinholeSet<CalibratedCamera>::Result actual = set.triangulateSafe(z);
}

/* ************************************************************************* */
// Cal3Bundler test
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
TEST(PinholeSet, Pinhole) {
  typedef PinholeCamera<Cal3Bundler> Camera;
  typedef vector<Point2> ZZ;
  PinholeSet<Camera> set;
  Camera camera;
  set.push_back(camera);
  set.push_back(camera);
  Point3 p(0, 0, 1);
  EXPECT(assert_equal(set, set));
  PinholeSet<Camera> set2 = set;
  set2.push_back(camera);
  EXPECT(!set.equals(set2));

  // Check measurements
  Point2 expected;
  ZZ z = set.project2(p);
  EXPECT(assert_equal(expected, z[0]));
  EXPECT(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix43 actualE;
  Matrix F1;
  {
    Matrix23 E1;
    Matrix23 H1;
    camera.project2(p, F1, E1);
    actualE << E1, E1;
  }

  // Check computed derivatives
  PinholeSet<Camera>::FBlocks F;
  Matrix E, H;
  set.project2(p, F, E);
  LONGS_EQUAL(2, F.size());
  EXPECT(assert_equal(F1, F[0]));
  EXPECT(assert_equal(F1, F[1]));
  EXPECT(assert_equal(actualE, E));

  // Check errors
  ZZ measured;
  measured.push_back(Point2(1, 2));
  measured.push_back(Point2(3, 4));
  Vector4 expectedV;

  // reprojectionError
  expectedV << -1, -2, -3, -4;
  Vector actualV = set.reprojectionError(p, measured);
  EXPECT(assert_equal(expectedV, actualV));

  // reprojectionErrorAtInfinity
  EXPECT(
      assert_equal(Point3(0, 0, 1),
          camera.backprojectPointAtInfinity(Point2())));
  actualV = set.reprojectionErrorAtInfinity(p, measured);
  EXPECT(assert_equal(expectedV, actualV));

  // Instantiate triangulateSafe
  TriangulationParameters params;
  TriangulationResult actual = set.triangulateSafe(z,params);
  CHECK(actual.degenerate());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

