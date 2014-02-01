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
#include <gtsam/geometry/Rot2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace gtsam;
using namespace GeographicLib;

/**
 * Factor to estimate rotation given magnetometer reading
 * This version uses model measured bM = scale * bRn * direction + bias
 * and assumes scale, direction, and the bias are given
 */
class MagFactor: public NoiseModelFactor1<Rot2> {

  const Vector3 measured_; /** The measured magnetometer values */
  const double scale_;
  const Sphere2 direction_;
  const Vector3 bias_;

public:

  /** Constructor */
  MagFactor(Key key, const Vector3& measured, const LieScalar& scale,
      const Sphere2& direction, const LieVector& bias,
      const SharedNoiseModel& model) :
      NoiseModelFactor1<Rot2>(model, key), //
      measured_(measured), scale_(scale), direction_(direction), bias_(bias) {
  }

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new MagFactor(*this)));
  }

  static Sphere2 unrotate(const Rot2& R, const Sphere2& p,
      boost::optional<Matrix&> HR = boost::none) {
    Sphere2 q = Rot3::yaw(R.theta()) * p;
    if (HR) // 2*3                     3*1
      (*HR) = -q.basis().transpose() * q.skew().col(2);
    return q;
  }

  /**
   * @brief vector of errors
   */
  Vector evaluateError(const Rot2& nRb,
      boost::optional<Matrix&> H = boost::none) const {
    // measured bM = nRb’ * nM + b, where b is unknown bias
    Sphere2 rotated = unrotate(nRb, direction_, H);
    Vector3 hx = scale_ * rotated.unitVector() + bias_;
    if (H) // I think H2 is 2*2, but we need 3*2
    {
      Matrix U;
      rotated.unitVector(U);
      *H = scale_ * U * (*H);
    }
    return hx - measured_;
  }
};

// *************************************************************************
// Convert from Mag to ENU
// ENU Origin is where the plane was in hold next to runway
// const double lat0 = 33.86998, lon0 = -84.30626, h0 = 274;

// Get field from http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
// Declination = -4.94 degrees (West), Inclination = 62.78 degrees Down
// As NED vector, in nT:
Vector3 nM(22653.29982, -1956.83010, 44202.47862);
// Let's assume scale factor,
double scale = 255.0 / 50000.0;
// ...ground truth orientation,
Rot3 nRb = Rot3::yaw(-0.1);
Rot2 theta = -nRb.yaw();
// ...and bias
Vector3 bias(10, -10, 50);
// ... then we measure
Vector3 scaled = scale * nM;
Vector3 measured = scale * nRb.transpose() * nM + bias;

LieScalar s(scale * nM.norm());
Sphere2 dir(nM[0], nM[1], nM[2]);

SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);

using boost::none;

// *************************************************************************
TEST( MagFactor, unrotate ) {
  Matrix H;
  Sphere2 expected(0.457383, 0.00632703, 0.889247);
  EXPECT( assert_equal(expected, MagFactor::unrotate(theta,dir,H),1e-5));
  EXPECT( assert_equal(numericalDerivative11<Sphere2,Rot2> //
      (boost::bind(&MagFactor::unrotate, _1, dir, none), theta), H, 1e-7));
}

// *************************************************************************
TEST( MagFactor, Factors ) {

  Matrix H1, H2, H3;

  // MagFactor
  MagFactor f(1, measured, s, dir, bias, model);
  EXPECT( assert_equal(zero(3),f.evaluateError(theta,H1),1e-5));
  EXPECT( assert_equal(numericalDerivative11<Rot2> //
      (boost::bind(&MagFactor::evaluateError, &f, _1, none), theta), H1, 1e-7));

// MagFactor1
  MagFactor1 f1(1, measured, s, dir, bias, model);
  EXPECT( assert_equal(zero(3),f1.evaluateError(nRb,H1),1e-5));
  EXPECT( assert_equal(numericalDerivative11<Rot3> //
      (boost::bind(&MagFactor1::evaluateError, &f1, _1, none), nRb), H1, 1e-7));

// MagFactor2
  MagFactor2 f2(1, 2, measured, nRb, model);
  EXPECT( assert_equal(zero(3),f2.evaluateError(scaled,bias,H1,H2),1e-5));
  EXPECT( assert_equal(numericalDerivative11<LieVector> //
      (boost::bind(&MagFactor2::evaluateError, &f2, _1, bias, none, none), scaled),//
      H1, 1e-7));
  EXPECT( assert_equal(numericalDerivative11<LieVector> //
      (boost::bind(&MagFactor2::evaluateError, &f2, scaled, _1, none, none), bias),//
      H2, 1e-7));

// MagFactor2
  MagFactor3 f3(1, 2, 3, measured, nRb, model);
  EXPECT(assert_equal(zero(3),f3.evaluateError(s,dir,bias,H1,H2,H3),1e-5));
  EXPECT(assert_equal(numericalDerivative11<LieScalar> //
      (boost::bind(&MagFactor3::evaluateError, &f3, _1, dir, bias, none, none, none), s),//
      H1, 1e-7));
  EXPECT(assert_equal(numericalDerivative11<Sphere2> //
      (boost::bind(&MagFactor3::evaluateError, &f3, s, _1, bias, none, none, none), dir),//
      H2, 1e-7));
  EXPECT(assert_equal(numericalDerivative11<LieVector> //
      (boost::bind(&MagFactor3::evaluateError, &f3, s, dir, _1, none, none, none), bias),//
      H3, 1e-7));
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
