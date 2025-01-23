/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGPSFactor.cpp
 * @brief   Unit test for GPSFactor
 * @author  Frank Dellaert
 * @date   January 22, 2014
 */

#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <GeographicLib/Config.h>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace gtsam;
using namespace GeographicLib;

#if GEOGRAPHICLIB_VERSION_MINOR<37
static const auto& kWGS84 = Geocentric::WGS84;
#else
static const auto& kWGS84 = Geocentric::WGS84();
#endif

// *************************************************************************
namespace example {
// ENU Origin is where the plane was in hold next to runway
static constexpr double lat0 = 33.86998, lon0 = -84.30626, h0 = 274;

// Convert from GPS to ENU
static const LocalCartesian origin_ENU(lat0, lon0, h0, kWGS84);

// Dekalb-Peachtree Airport runway 2L
static constexpr double lat = 33.87071, lon = -84.30482, h = 274;

// Random lever arm
static const Point3 leverArm(0.1, 0.2, 0.3);
}  // namespace example

// *************************************************************************
TEST( GPSFactor, Constructor ) {
  using namespace example;

  // From lat-lon to geocentric
  double E, N, U;
  origin_ENU.Forward(lat, lon, h, E, N, U);
  EXPECT_DOUBLES_EQUAL(133.24, E, 1e-2);
  EXPECT_DOUBLES_EQUAL(80.98, N, 1e-2);
  EXPECT_DOUBLES_EQUAL(0, U, 1e-2);

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactor factor(key, Point3(E, N, U), model);

  // Create a linearization point at zero error
  Pose3 T(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(E, N, U));
  EXPECT(assert_equal(Z_3x1,factor.evaluateError(T),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative11<Vector, Pose3>(
      [&factor](const Pose3& T) { return factor.evaluateError(T); }, T);

  // Use the factor to calculate the derivative
  Matrix actualH;
  factor.evaluateError(T, actualH);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

// *************************************************************************
TEST( GPSFactorArm, Constructor ) {
  using namespace example;

  // From lat-lon to geocentric
  double E, N, U;
  origin_ENU.Forward(lat, lon, h, E, N, U);

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactorArm factor(key, Point3(E, N, U), leverArm, model);

  // Create a linearization point at zero error
  const Rot3 nRb = Rot3::RzRyRx(0.15, -0.30, 0.45);
  const Point3 np = Point3(E, N, U) - nRb * leverArm;
  Pose3 T(nRb, np);
  EXPECT(assert_equal(Z_3x1,factor.evaluateError(T),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative11<Vector, Pose3>(
      [&factor](const Pose3& T) { return factor.evaluateError(T); }, T);

  // Use the factor to calculate the derivative
  Matrix actualH;
  factor.evaluateError(T, actualH);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

// *************************************************************************
TEST( GPSFactorArmCalib, Constructor ) {
  using namespace example;

  // From lat-lon to geocentric
  double E, N, U;
  origin_ENU.Forward(lat, lon, h, E, N, U);

  // Factor
  Key key1(1), key2(2);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactorArmCalib factor(key1, key2, Point3(E, N, U), model);

  // Create a linearization point at zero error
  const Rot3 nRb = Rot3::RzRyRx(0.15, -0.30, 0.45);
  const Point3 np = Point3(E, N, U) - nRb * leverArm;
  Pose3 T(nRb, np);
  EXPECT(assert_equal(Z_3x1,factor.evaluateError(T, leverArm),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      [&factor](const Pose3& pose_arg) {
        return factor.evaluateError(pose_arg, leverArm);
      },
      T);
  Matrix expectedH2 = numericalDerivative11<Vector, Point3>(
      [&factor, &T](const Point3& point_arg) {
        return factor.evaluateError(T, point_arg);
      },
      leverArm);

  // Use the factor to calculate the derivative
  Matrix actualH1, actualH2;
  factor.evaluateError(T, leverArm, actualH1, actualH2);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
}

// *************************************************************************
TEST( GPSFactor2, Constructor ) {
  using namespace example;

  // From lat-lon to geocentric
  double E, N, U;
  origin_ENU.Forward(lat, lon, h, E, N, U);

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactor2 factor(key, Point3(E, N, U), model);

  // Create a linearization point at zero error
  NavState T(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(E, N, U), Vector3::Zero());
  EXPECT(assert_equal(Z_3x1,factor.evaluateError(T),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative11<Vector, NavState>(
      [&factor](const NavState& T) { return factor.evaluateError(T); }, T);

  // Use the factor to calculate the derivative
  Matrix actualH;
  factor.evaluateError(T, actualH);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

// *************************************************************************
TEST( GPSFactor2Arm, Constructor ) {
  using namespace example;

  // From lat-lon to geocentric
  double E, N, U;
  origin_ENU.Forward(lat, lon, h, E, N, U);

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactor2Arm factor(key, Point3(E, N, U), leverArm, model);

  // Create a linearization point at zero error
  const Rot3 nRb = Rot3::RzRyRx(0.15, -0.30, 0.45);
  const Point3 np = Point3(E, N, U) - nRb * leverArm;
  NavState T(nRb, np, Vector3::Zero());
  EXPECT(assert_equal(Z_3x1,factor.evaluateError(T),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative11<Vector, NavState>(
      [&factor](const NavState& T) { return factor.evaluateError(T); }, T);

  // Use the factor to calculate the derivative
  Matrix actualH;
  factor.evaluateError(T, actualH);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

// *************************************************************************
TEST( GPSFactor2ArmCalib, Constructor ) {
  using namespace example;

  // From lat-lon to geocentric
  double E, N, U;
  origin_ENU.Forward(lat, lon, h, E, N, U);

  // Factor
  Key key1(1), key2(2);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactor2ArmCalib factor(key1, key2, Point3(E, N, U), model);

  // Create a linearization point at zero error
  const Rot3 nRb = Rot3::RzRyRx(0.15, -0.30, 0.45);
  const Point3 np = Point3(E, N, U) - nRb * leverArm;
  NavState T(nRb, np, Vector3::Zero());
  EXPECT(assert_equal(Z_3x1,factor.evaluateError(T, leverArm),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector, NavState>(
      [&factor](const NavState& nav_arg) {
        return factor.evaluateError(nav_arg, leverArm);
      },
      T);
  Matrix expectedH2 = numericalDerivative11<Vector, Point3>(
      [&factor, &T](const Point3& point_arg) {
        return factor.evaluateError(T, point_arg);
      },
      leverArm);

  // Use the factor to calculate the derivative
  Matrix actualH1, actualH2;
  factor.evaluateError(T, leverArm, actualH1, actualH2);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
}

//***************************************************************************
TEST(GPSData, init) {

  // GPS Reading 1 will be ENU origin
  double t1 = 84831;
  Point3 NED1(0, 0, 0);
  LocalCartesian enu(35.4393283333333, -119.062986666667, 275.54, kWGS84);

  // GPS Readin 2
  double t2 = 84831.5;
  double E, N, U;
  enu.Forward(35.4394633333333, -119.063146666667, 276.52, E, N, U);
  Point3 NED2(N, E, -U);

  // Estimate initial state
  const auto [T, nV] = GPSFactor::EstimateState(t1, NED1, t2, NED2, 84831.0796);

  // Check values values
  EXPECT(assert_equal((Vector )Vector3(29.9575, -29.0564, -1.95993), nV, 1e-4));
  EXPECT( assert_equal(Rot3::Ypr(-0.770131, 0.046928, 0), T.rotation(), 1e-5));
  Point3 expectedT(2.38461, -2.31289, -0.156011);
  EXPECT(assert_equal(expectedT, T.translation(), 1e-5));
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
