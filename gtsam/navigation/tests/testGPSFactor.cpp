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
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// *************************************************************************
TEST( GPSFactor, Constructors ) {
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  // TODO: convert from GPS to UTM
  // GPS: -84.30482,33.87071,274
  // UTM: 45N 250694.42 3751090.08 (Eastings, Northings)
  // We choose ENU coordinate system with origin 250000 3751000
  Point3 enu(694.42, 90.08, 274);
  GPSFactor factor1(key, enu, model);
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
