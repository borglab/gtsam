/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testMagFactor.cpp
 * @brief   Unit test for MagFactor
 * @author  Frank Dellaert
 * @date   January 29, 2014
 */

#include <gtsam/navigation/MagFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/bind/bind.hpp>

#include <CppUnitLite/TestHarness.h>

#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace gtsam;
using namespace GeographicLib;

namespace {
// Convert from Mag to ENU
// ENU Origin is where the plane was in hold next to runway
// const double lat0 = 33.86998, lon0 = -84.30626, h0 = 274;

// Get field from http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
// Declination = -4.94 degrees (West), Inclination = 62.78 degrees Down
// As NED vector, in nT:
Point3 nM(22653.29982, -1956.83010, 44202.47862);
// Let's assume scale factor,
double scale = 255.0 / 50000.0;
// ...ground truth orientation,
Rot3 nRb = Rot3::Yaw(-0.1);
Rot2 theta = nRb.yaw();
// ...and bias
Point3 bias(10, -10, 50);
// ... then we measure
Point3 scaled = scale * nM;
Point3 measured = nRb.inverse() * (scale * nM) + bias;

double s(scale* nM.norm());
Unit3 dir(nM);

SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
}  // namespace

// *************************************************************************
TEST( MagFactor, unrotate ) {
  Matrix H;
  Point3 expected(22735.5, 314.502, 44202.5);
  EXPECT( assert_equal(expected, MagFactor::unrotate(theta,nM,&H),1e-1));
  EXPECT(assert_equal(numericalDerivative11<Point3, Rot2> //
      (std::bind(&MagFactor::unrotate, std::placeholders::_1, nM, OptionalNone), theta), H, 1e-6));
}

// *************************************************************************
TEST( MagFactor, Factors ) {

  Matrix H1, H2, H3;

  // MagFactor
  MagFactor f(1, measured, s, dir, bias, model);
  EXPECT(assert_equal(Z_3x1,f.evaluateError(theta,H1),1e-5));
  EXPECT(assert_equal((Matrix)numericalDerivative11<Vector, Rot2>
        ([&f] (const Rot2& theta) { return f.evaluateError(theta); }, theta), H1, 1e-7));

  // MagFactor1
  MagFactor1 f1(1, measured, s, dir, bias, model);
  EXPECT(assert_equal(Z_3x1,f1.evaluateError(nRb,H1),1e-5));
  EXPECT(assert_equal(numericalDerivative11<Vector, Rot3>
      ([&f1] (const Rot3& nRb) { return f1.evaluateError(nRb); }, nRb), H1, 1e-7));

  // MagFactor2
  MagFactor2 f2(1, 2, measured, nRb, model);
  EXPECT(assert_equal(Z_3x1,f2.evaluateError(scaled,bias,H1,H2),1e-5));
  EXPECT(assert_equal(numericalDerivative11<Vector, Point3> //
      ([&f2] (const Point3& scaled) { return f2.evaluateError(scaled,bias); }, scaled), H1, 1e-7));
  EXPECT(assert_equal(numericalDerivative11<Vector, Point3> //
      ([&f2] (const Point3& bias) { return f2.evaluateError(scaled,bias); }, bias), H2, 1e-7));

  // MagFactor3
  MagFactor3 f3(1, 2, 3, measured, nRb, model);
  EXPECT(assert_equal(Z_3x1,f3.evaluateError(s,dir,bias,H1,H2,H3),1e-5));
  EXPECT(assert_equal((Matrix)numericalDerivative11<Vector,double> //
      ([&f3] (double s) { return f3.evaluateError(s,dir,bias); }, s), H1, 1e-7));
  EXPECT(assert_equal(numericalDerivative11<Vector,Unit3> //
      ([&f3] (const Unit3& dir) { return f3.evaluateError(s,dir,bias); }, dir), H2, 1e-7));
  EXPECT(assert_equal(numericalDerivative11<Vector,Point3> //
      ([&f3] (const Point3& bias) { return f3.evaluateError(s,dir,bias); }, bias), H3, 1e-7));
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
