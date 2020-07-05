/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3DFisheye.cpp
 * @brief Unit tests for fisheye calibration class
 * @author ghaggin
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Point3.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3Fisheye)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3Fisheye)

static const double fx = 250, fy = 260, s = 0.1, u0 = 320, v0 = 240;
static Cal3Fisheye K(fx, fy, s, u0, v0, -0.013721808247486035,
                     0.020727425669427896, -0.012786476702685545,
                     0.0025242267320687625);
static Point2 kTestPoint2(2, 3);

/* ************************************************************************* */
TEST(Cal3Fisheye, assert_equal) { CHECK(assert_equal(K, K, 1e-5)); }

/* ************************************************************************* */
TEST(Cal3Fisheye, retract) {
  Cal3Fisheye expected(K.fx() + 1, K.fy() + 2, K.skew() + 3, K.px() + 4,
                       K.py() + 5, K.k1() + 6, K.k2() + 7, K.k3() + 8,
                       K.k4() + 9);
  Vector d(9);
  d << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Cal3Fisheye actual = K.retract(d);
  CHECK(assert_equal(expected, actual, 1e-7));
  CHECK(assert_equal(d, K.localCoordinates(actual), 1e-7));
}

/* ************************************************************************* */
TEST(Cal3Fisheye, uncalibrate1) {
  // Calculate the solution
  const double xi = kTestPoint2.x(), yi = kTestPoint2.y();
  const double r = sqrt(xi * xi + yi * yi);
  const double t = atan(r);
  const double tt = t * t, t4 = tt * tt, t6 = tt * t4, t8 = t4 * t4;
  const double td =
      t * (1 + K.k1() * tt + K.k2() * t4 + K.k3() * t6 + K.k4() * t8);
  Vector3 pd(td / r * xi, td / r * yi, 1);
  Vector3 v = K.K() * pd;

  Point2 uv_sol(v[0] / v[2], v[1] / v[2]);

  Point2 uv = K.uncalibrate(kTestPoint2);
  CHECK(assert_equal(uv, uv_sol));
}

/* ************************************************************************* */
// For numerical derivatives
Point2 f(const Cal3Fisheye& k, const Point2& pt) { return k.uncalibrate(pt); }

/* ************************************************************************* */
TEST(Cal3Fisheye, Derivatives) {
  Matrix H1, H2;
  K.uncalibrate(kTestPoint2, H1, H2);
  CHECK(assert_equal(numericalDerivative21(f, K, kTestPoint2, 1e-7), H1, 1e-5));
  CHECK(assert_equal(numericalDerivative22(f, K, kTestPoint2, 1e-7), H2, 1e-5));
}

/* ************************************************************************* */
// Check that a point at (0,0) projects to the image center.
TEST(Cal3Fisheye, uncalibrate2) {
  Point2 pz(0, 0);
  Matrix H1, H2;
  auto uv = K.uncalibrate(pz, H1, H2);
  CHECK(assert_equal(uv, Point2(u0, v0)));
  CHECK(assert_equal(numericalDerivative21(f, K, pz, 1e-7), H1, 1e-5));
  // TODO(frank): the second jacobian is all NaN for the image center!
  // CHECK(assert_equal(numericalDerivative22(f, K, pz, 1e-7), H2, 1e-5));
}

/* ************************************************************************* */
//  This test uses cv2::fisheye::projectPoints to test that uncalibrate
//  properly projects a point into the image plane.  One notable difference
//  between opencv and the Cal3Fisheye::uncalibrate function is the skew
//  parameter. The equivalence is alpha = s/fx.
//
// Python script to project points with fisheye model in OpenCv
// (script run with OpenCv version 4.2.0 and Numpy version 1.18.2)
// clang-format off
/*
===========================================================

import numpy as np
import cv2

objpts = np.float64([[23,27,31]]).reshape(1,-1,3)

cameraMatrix = np.float64([
    [250, 0, 320],
    [0, 260, 240],
    [0,0,1]
])
alpha = 0.1/250
distCoeffs = np.float64([-0.013721808247486035, 0.020727425669427896,-0.012786476702685545, 0.0025242267320687625]) 

rvec = np.float64([[0.,0.,0.]])
tvec = np.float64([[0.,0.,0.]]);
imagePoints, jacobian = cv2.fisheye.projectPoints(objpts, rvec, tvec, cameraMatrix, distCoeffs, alpha=alpha) 
np.set_printoptions(precision=14) 
print(imagePoints)

===========================================================
 * Script output: [[[457.82638130304935 408.18905848512986]]]
 */
// clang-format on
TEST(Cal3Fisheye, uncalibrate3) {
  Point3 p3(23, 27, 31);
  Point2 xi(p3.x() / p3.z(), p3.y() / p3.z());
  auto uv = K.uncalibrate(xi);
  CHECK(assert_equal(uv, Point2(457.82638130304935, 408.18905848512986)));
}

/* ************************************************************************* */
TEST(Cal3Fisheye, calibrate1) {
  Point2 pi;
  Point2 uv;
  Point2 pi_hat;

  pi = Point2(0.5, 0.5);     // point in intrinsic coordinates
  uv = K.uncalibrate(pi);    // map intrinsic coord to image plane (pi)
  pi_hat = K.calibrate(uv);  // map image coords (pi) back to intrinsic coords
  CHECK(traits<Point2>::Equals(pi, pi_hat,
                               1e-5));  // check that the inv mapping works

  pi = Point2(-0.7, -1.2);
  uv = K.uncalibrate(pi);
  pi_hat = K.calibrate(uv);
  CHECK(traits<Point2>::Equals(pi, pi_hat, 1e-5));

  pi = Point2(-3, 5);
  uv = K.uncalibrate(pi);
  pi_hat = K.calibrate(uv);
  CHECK(traits<Point2>::Equals(pi, pi_hat, 1e-5));

  pi = Point2(7, -12);
  uv = K.uncalibrate(pi);
  pi_hat = K.calibrate(uv);
  CHECK(traits<Point2>::Equals(pi, pi_hat, 1e-5));
}

/* ************************************************************************* */
// Check that calibrate returns (0,0) for the image center
TEST(Cal3Fisheye, calibrate2) {
  Point2 uv(u0, v0);
  auto xi_hat = K.calibrate(uv);
  CHECK(assert_equal(xi_hat, Point2(0, 0)))
}

/* ************************************************************************* */
// Run calibrate on OpenCv test from uncalibrate3
//  (script shown above)
// 3d point: (23, 27, 31)
// 2d point in image plane: (457.82638130304935, 408.18905848512986)
TEST(Cal3Fisheye, calibrate3) {
  Point3 p3(23, 27, 31);
  Point2 xi(p3.x() / p3.z(), p3.y() / p3.z());
  Point2 uv(457.82638130304935, 408.18905848512986);
  auto xi_hat = K.calibrate(uv);
  CHECK(assert_equal(xi_hat, xi));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
