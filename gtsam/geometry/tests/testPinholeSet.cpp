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

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
#include <gtsam/geometry/CalibratedCamera.h>
TEST(PinholeSet, Stereo) {
  typedef Point2Vector ZZ;
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
  PinholeSet<CalibratedCamera>::FBlocks Fs;
  Matrix E;
  set.project2(p, Fs, E);
  LONGS_EQUAL(2, Fs.size());
  EXPECT(assert_equal(F1, Fs[0]));
  EXPECT(assert_equal(F1, Fs[1]));
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
  typedef Point2Vector ZZ;
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
  Point2 expected(0,0);
  ZZ z = set.project2(p);
  EXPECT(assert_equal(expected, z[0]));
  EXPECT(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix actualE;
  Matrix F1;
  {
    Matrix23 E1;
    camera.project2(p, F1, E1);
    actualE.resize(4, 3);
    actualE << E1, E1;
  }

  // Check computed derivatives
  {
    PinholeSet<Camera>::FBlocks Fs;
    Matrix E;
    set.project2(p, Fs, E);
    EXPECT(assert_equal(actualE, E));
    LONGS_EQUAL(2, Fs.size());
    EXPECT(assert_equal(F1, Fs[0]));
    EXPECT(assert_equal(F1, Fs[1]));
  }

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
  Unit3 pointAtInfinity(0, 0, 1000);
  {
    Matrix22 E1;
    camera.project2(pointAtInfinity, F1, E1);
    actualE.resize(4, 2);
    actualE << E1, E1;
  }
  EXPECT(
      assert_equal(pointAtInfinity,
          camera.backprojectPointAtInfinity(Point2(0,0))));
  {
    PinholeSet<Camera>::FBlocks Fs;
    Matrix E;
    actualV = set.reprojectionError(pointAtInfinity, measured, Fs, E);
    EXPECT(assert_equal(actualE, E));
    LONGS_EQUAL(2, Fs.size());
    EXPECT(assert_equal(F1, Fs[0]));
    EXPECT(assert_equal(F1, Fs[1]));
  }
  EXPECT(assert_equal(expectedV, actualV));

  // Instantiate triangulateSafe
  TriangulationParameters params;
  TriangulationResult actual = set.triangulateSafe(z, params);
  CHECK(actual.degenerate());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

