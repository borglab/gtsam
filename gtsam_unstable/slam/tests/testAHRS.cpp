/*
 * @file testAHRS.cpp
 * @brief Test AHRS
 * @author Frank Dellaert
 * @author Chris Beall
 */

#include "../AHRS.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>
#include <list>

using namespace std;
using namespace gtsam;

// stationary interval of gyro U and acc F
Matrix stationaryU = trans((Matrix(3, 3) << -0.0004,-0.0002,-0.0014,0.0006,-0.0003,0.0007,0.0006,-0.0002,-0.0003).finished());
Matrix stationaryF = trans((Matrix(3, 3) << 0.1152,-0.0188,9.7419,-0.0163,0.0146,9.7753,-0.0283,-0.0428,9.9021).finished());
double g_e = 9.7963; // Atlanta

/* ************************************************************************* */
TEST (AHRS, cov) {

  // samples stored by row
  Matrix A = (Matrix(4, 3) <<
      1.0, 2.0, 3.0,
      5.0, 7.0, 0.0,
      9.0, 4.0, 7.0,
      6.0, 3.0, 2.0).finished();

  Matrix actual = AHRS::Cov(trans(A));
  Matrix expected = (Matrix(3, 3) <<
      10.9167,    2.3333,    5.0000,
          2.3333,    4.6667,   -2.6667,
          5.0000,   -2.6667,    8.6667).finished();

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST (AHRS, covU) {

  Matrix actual = AHRS::Cov(10000*stationaryU);
  Matrix expected = (Matrix(3, 3) <<
      33.3333333,    -1.66666667,    53.3333333,
      -1.66666667,    0.333333333,   -5.16666667,
      53.3333333,  -5.16666667,    110.333333).finished();

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST (AHRS, covF) {

  Matrix actual = AHRS::Cov(100*stationaryF);
  Matrix expected = (Matrix(3, 3) <<
      63.3808333, -0.432166667,  -48.1706667,
    -0.432166667,   8.31053333,  -16.6792667,
     -48.1706667,  -16.6792667,   71.4297333).finished();

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST (AHRS, constructor) {
  AHRS ahrs = AHRS(stationaryU,stationaryF,g_e);
}

/* ************************************************************************* */
// TODO make a testMechanization_bRn2
TEST(AHRS, Mechanization_integrate) {
  AHRS ahrs = AHRS(stationaryU, stationaryF, g_e);
  // const auto [mech, state] = ahrs.initialize(g_e);
  // Vector u = Vector3(0.05, 0.0, 0.0);
  // double dt = 2;
  // Rot3 expected;
  // Mechanization_bRn2 mech2 = mech.integrate(u, dt);
  // Rot3 actual = mech2.bRn();
  // EXPECT(assert_equal(expected, actual));
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

