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

#include <GeographicLib/Geocentric.hpp>
#include <CppUnitLite/TestHarness.h>

using namespace std;
//using namespace gtsam;
using namespace GeographicLib;

/* ************************************************************************* */

TEST( GeographicLib, Geocentric) {

  Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

  // Dekalb-Peachtree Airport runway 2L
  double lat = 33.87071, lon = -84.30482000000001, h = 274;

  // UTM is 45N 250694.42 3751090.08
  // Obtained by
  // http://geographiclib.sourceforge.net/cgi-bin/GeoConvert?input=33.87071+84.30482000000001&zone=-3&prec=2&option=Submit

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
