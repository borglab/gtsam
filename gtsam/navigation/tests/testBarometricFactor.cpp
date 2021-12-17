/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testBarometricFactor.cpp
 * @brief   Unit test for BarometricFactor
 * @author  Peter Milani
 * @date   16 Dec, 2021
 */

#include <gtsam/navigation/BarometricFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/bind/bind.hpp>

#include <CppUnitLite/TestHarness.h>


using namespace std::placeholders;
using namespace std;
using namespace gtsam;


// *************************************************************************
namespace example {
}

double metersToBaro(const double& meters)
{
    double temp = 15.04 - 0.00649*meters;
    return 101.29*std::pow(((temp+273.1)/288.08), 5.256);

}

// *************************************************************************
TEST( BarometricFactor, Constructor ) {
  using namespace example;

  //meters to barometric.

  double baroMeasurement = metersToBaro(10.);

  // Factor
  Key key(1);
  Key key2(2);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(1, 0.25);
  BarometricFactor factor(key, key2, baroMeasurement, model);

  // Create a linearization point at zero error
  Pose3 T(Rot3::RzRyRx(0., 0., 0.), Point3(0., 0., 10.));
  double baroBias=0.;
  Vector1 zero;
  zero<< 0.;
  EXPECT(assert_equal(zero, factor.evaluateError(T, baroBias),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative21<Vector,Pose3, double>(
      std::bind(&BarometricFactor::evaluateError, &factor, std::placeholders::_1,
          std::placeholders::_2, boost::none, boost::none), T, baroBias);

  Matrix expectedH2 = numericalDerivative22<Vector,Pose3, double>(
      std::bind(&BarometricFactor::evaluateError, &factor, std::placeholders::_1,
          std::placeholders::_2, boost::none, boost::none), T, baroBias);


  // Use the factor to calculate the derivative
  Matrix actualH, actualH2;
  factor.evaluateError(T, baroBias,  actualH, actualH2);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
}

// *************************************************************************

//***************************************************************************
TEST(GPSData, init) {

  /* GPS Reading 1 will be ENU origin
  double t1 = 84831;
  Point3 NED1(0, 0, 0);
  LocalCartesian enu(35.4393283333333, -119.062986666667, 275.54, kWGS84);

  // GPS Readin 2
  double t2 = 84831.5;
  double E, N, U;
  enu.Forward(35.4394633333333, -119.063146666667, 276.52, E, N, U);
  Point3 NED2(N, E, -U);

  // Estimate initial state
  Pose3 T;
  Vector3 nV;
  boost::tie(T, nV) = GPSFactor::EstimateState(t1, NED1, t2, NED2, 84831.0796);

  // Check values values
  EXPECT(assert_equal((Vector )Vector3(29.9575, -29.0564, -1.95993), nV, 1e-4));
  EXPECT( assert_equal(Rot3::Ypr(-0.770131, 0.046928, 0), T.rotation(), 1e-5));
  Point3 expectedT(2.38461, -2.31289, -0.156011);
  EXPECT(assert_equal(expectedT, T.translation(), 1e-5));
  */
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
