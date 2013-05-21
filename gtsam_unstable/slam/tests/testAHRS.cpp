/*
 * @file testAHRS.cpp
 * @brief Test AHRS
 * @author Frank Dellaert
 * @author Chris Beall
 */

#include <list>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>
#include "../AHRS.h"
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// stationary interval of gyro U and acc F
Matrix stationaryU = trans(Matrix_(3,3,-0.0004,-0.0002,-0.0014,0.0006,-0.0003,0.0007,0.0006,-0.0002,-0.0003));
Matrix stationaryF = trans(Matrix_(3,3,0.1152,-0.0188,9.7419,-0.0163,0.0146,9.7753,-0.0283,-0.0428,9.9021));
double g_e = 9.7963; // Atlanta

/* ************************************************************************* */
TEST (AHRS, cov) {

  // samples stored by row
  Matrix A = Matrix_(4, 3,
      1.0, 2.0, 3.0,
      5.0, 7.0, 0.0,
      9.0, 4.0, 7.0,
      6.0, 3.0, 2.0);

  Matrix actual = cov(trans(A));
  Matrix expected = Matrix_(3, 3,
      10.9167,    2.3333,    5.0000,
          2.3333,    4.6667,   -2.6667,
          5.0000,   -2.6667,    8.6667);

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST (AHRS, covU) {

  Matrix actual = cov(10000*stationaryU);
  Matrix expected = Matrix_(3, 3,
      33.3333333,    -1.66666667,    53.3333333,
      -1.66666667,    0.333333333,   -5.16666667,
      53.3333333,  -5.16666667,    110.333333);

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST (AHRS, covF) {

  Matrix actual = cov(100*stationaryF);
  Matrix expected = Matrix_(3, 3,
      63.3808333, -0.432166667,  -48.1706667,
    -0.432166667,   8.31053333,  -16.6792667,
     -48.1706667,  -16.6792667,   71.4297333);

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST (AHRS, constructor) {
  AHRS ahrs = AHRS(stationaryU,stationaryF,g_e);
}

/* ************************************************************************* */
/* TODO: currently fails because of problem with ill-conditioned system
TEST (AHRS, init) {
  AHRS ahrs = AHRS(stationaryU,stationaryF,g_e);
  std::pair<Mechanization_bRn2, KalmanFilter::State> result =  ahrs.initialize(g_e);
}
*/
/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

