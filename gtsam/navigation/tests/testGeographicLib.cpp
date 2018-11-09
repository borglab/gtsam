/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGeographicLib.cpp
 * @brief   Unit tests for coordinate conversions
 * @author  Frank Dellaert
 */

#include <GeographicLib/Config.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <gtsam/base/types.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/algorithm/string.hpp>
#include <string>
#include <iostream>

using namespace std;
//using namespace gtsam;
using namespace GeographicLib;

// Dekalb-Peachtree Airport runway 2L
static const double lat = 33.87071, lon = -84.30482, h = 274;

#if GEOGRAPHICLIB_VERSION_MINOR<37
static const auto& kWGS84 = Geocentric::WGS84;
#else
static const auto& kWGS84 = Geocentric::WGS84();
#endif

//**************************************************************************
TEST( GeographicLib, Geocentric) {

  // From lat-lon to geocentric
  double X, Y, Z;
  kWGS84.Forward(lat, lon, h, X, Y, Z);
  EXPECT_DOUBLES_EQUAL(526, X/1000, 1);
  EXPECT_DOUBLES_EQUAL(-5275, Y/1000, 1);
  EXPECT_DOUBLES_EQUAL(3535, Z/1000, 1);

  // From geocentric to lat-lon
  double lat_, lon_, h_;
  kWGS84.Reverse(X, Y, Z, lat_, lon_, h_);
  EXPECT_DOUBLES_EQUAL(lat, lat_, 1e-5);
  EXPECT_DOUBLES_EQUAL(lon, lon_, 1e-5);
  EXPECT_DOUBLES_EQUAL(h, h_, 1e-5);
}

//**************************************************************************
TEST( GeographicLib, UTM) {

  // From lat-lon to UTM
  int zone;
  bool northp;
  double x, y;
  UTMUPS::Forward(lat, lon, zone, northp, x, y);

  // UTM is 16N 749305.58 3751090.08
  // Obtained by
  // http://geographiclib.sourceforge.net/cgi-bin/GeoConvert?input=33.87071+-84.30482000000001&zone=-3&prec=2&option=Submit
  auto actual = UTMUPS::EncodeZone(zone, northp);
  boost::to_upper(actual);
  EXPECT(actual=="16N");
  EXPECT_DOUBLES_EQUAL(749305.58, x, 1e-2);
  EXPECT_DOUBLES_EQUAL(3751090.08, y, 1e-2);
}

//**************************************************************************
TEST( GeographicLib, ENU) {

  // ENU Origin is where the plane was in hold next to runway
  const double lat0 = 33.86998, lon0 = -84.30626, h0 = 274;
  LocalCartesian enu(lat0, lon0, h0, kWGS84);

  // From lat-lon to geocentric
  double E, N, U;
  enu.Forward(lat0, lon0, h0, E, N, U);
  EXPECT_DOUBLES_EQUAL(0, E, 1e-2);
  EXPECT_DOUBLES_EQUAL(0, N, 1e-2);
  EXPECT_DOUBLES_EQUAL(0, U, 1e-2);

  // From lat-lon to geocentric
  enu.Forward(lat, lon, h, E, N, U);
  EXPECT_DOUBLES_EQUAL(133.24, E, 1e-2);
  EXPECT_DOUBLES_EQUAL(80.98, N, 1e-2);
  EXPECT_DOUBLES_EQUAL(0, U, 1e-2);

  // From geocentric to lat-lon
  double lat_, lon_, h_;
  enu.Reverse(E, N, U, lat_, lon_, h_);
  EXPECT_DOUBLES_EQUAL(lat, lat_, 1e-5);
  EXPECT_DOUBLES_EQUAL(lon, lon_, 1e-5);
  EXPECT_DOUBLES_EQUAL(h, h_, 1e-5);
}

//**************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//**************************************************************************
