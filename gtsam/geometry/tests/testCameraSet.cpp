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
  typedef CameraSet<Camera> Set;
  typedef vector<Point2> ZZ;
  Set set;
  Camera camera;
  set.push_back(camera);
  set.push_back(camera);
  Point3 p(0, 0, 1);
  EXPECT(assert_equal(set, set));
  Set set2 = set;
  set2.push_back(camera);
  EXPECT(!set.equals(set2));

  // Check measurements
  Point2 expected(0,0);
  ZZ z = set.project2(p);
  EXPECT(assert_equal(expected, z[0]));
  EXPECT(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix actualE;
  Matrix29 F1;
  {
    Matrix23 E1;
    camera.project2(p, F1, E1);
    actualE.resize(4,3);
    actualE << E1, E1;
  }

  // Check computed derivatives
  Set::FBlocks Fs;
  Matrix E;
  set.project2(p, Fs, E);
  LONGS_EQUAL(2, Fs.size());
  EXPECT(assert_equal(F1, Fs[0]));
  EXPECT(assert_equal(F1, Fs[1]));
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

  // Check Schur complement
  Matrix F(4, 18);
  F << F1, Matrix29::Zero(), Matrix29::Zero(), F1;
  Matrix Ft = F.transpose();
  Matrix34 Et = E.transpose();
  Matrix3 P = Et * E;
  Matrix schur(19, 19);
  Vector4 b = actualV;
  Vector v = Ft * (b - E * P * Et * b);
  schur << Ft * F - Ft * E * P * Et * F, v, v.transpose(), 30;
  SymmetricBlockMatrix actualReduced = Set::SchurComplement(Fs, E, P, b);
  EXPECT(assert_equal(schur, actualReduced.selfadjointView()));

  // Check Schur complement update, same order, should just double
  FastVector<Key> allKeys, keys;
  allKeys.push_back(1);
  allKeys.push_back(2);
  keys.push_back(1);
  keys.push_back(2);
  Set::UpdateSchurComplement(Fs, E, P, b, allKeys, keys, actualReduced);
  EXPECT(assert_equal((Matrix )(2.0 * schur), actualReduced.selfadjointView()));

  // Check Schur complement update, keys reversed
  FastVector<Key> keys2;
  keys2.push_back(2);
  keys2.push_back(1);
  Set::UpdateSchurComplement(Fs, E, P, b, allKeys, keys2, actualReduced);
  Vector4 reverse_b;
  reverse_b << b.tail<2>(), b.head<2>();
  Vector reverse_v = Ft * (reverse_b - E * P * Et * reverse_b);
  Matrix A(19, 19);
  A << Ft * F - Ft * E * P * Et * F, reverse_v, reverse_v.transpose(), 30;
  EXPECT(assert_equal((Matrix )(2.0 * schur + A), actualReduced.selfadjointView()));

  // reprojectionErrorAtInfinity
  Unit3 pointAtInfinity(0, 0, 1000);
  EXPECT(
      assert_equal(pointAtInfinity,
          camera.backprojectPointAtInfinity(Point2(0,0))));
  actualV = set.reprojectionError(pointAtInfinity, measured, Fs, E);
  EXPECT(assert_equal(expectedV, actualV));
  LONGS_EQUAL(2, Fs.size());
  {
    Matrix22 E1;
    camera.project2(pointAtInfinity, F1, E1);
    actualE.resize(4,2);
    actualE << E1, E1;
  }
  EXPECT(assert_equal(F1, Fs[0]));
  EXPECT(assert_equal(F1, Fs[1]));
  EXPECT(assert_equal(actualE, E));
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
  ZZ z = set.project2(p);
  EXPECT(assert_equal(expected, z[0]));
  EXPECT(assert_equal(expected, z[1]));

  // Calculate expected derivatives using Pinhole
  Matrix63 actualE;
  Matrix F1;
  {
    Matrix33 E1;
    camera.project2(p, F1, E1);
    actualE << E1, E1;
  }

  // Check computed derivatives
  CameraSet<StereoCamera>::FBlocks Fs;
  Matrix E;
  set.project2(p, Fs, E);
  LONGS_EQUAL(2, Fs.size());
  EXPECT(assert_equal(F1, Fs[0]));
  EXPECT(assert_equal(F1, Fs[1]));
  EXPECT(assert_equal(actualE, E));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

