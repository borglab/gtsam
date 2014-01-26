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
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace GeographicLib;
using namespace gtsam;

// *************************************************************************
TEST( GPSFactor, Constructors ) {

  // Convert from GPS to ENU
  // ENU Origin is where the plane was in hold next to runway
  const double lat0 = 33.86998, lon0 = -84.30626, h0 = 274;
  LocalCartesian enu(lat0, lon0, h0, Geocentric::WGS84);

  // Dekalb-Peachtree Airport runway 2L
  const double lat = 33.87071, lon = -84.30482, h = 274;

  // From lat-lon to geocentric
  double E, N, U;
  enu.Forward(lat, lon, h, E, N, U);
  EXPECT_DOUBLES_EQUAL(133.24, E, 1e-2);
  EXPECT_DOUBLES_EQUAL(80.98, N, 1e-2);
  EXPECT_DOUBLES_EQUAL(0, U, 1e-2);

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactor factor(key, Point3(E, N, U), model);

  // Create a linearization point at the zero-error point
  Pose3 pose(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Pose3>(
      boost::bind(&GPSFactor::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative
  Matrix actualH1;
  factor.evaluateError(pose, actualH1);

  // Verify we get the expected error
  CHECK(assert_equal(expectedH1, actualH1, 1e-9));
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
