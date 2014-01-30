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
 * @date   January 22, 2014
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/LieScalar.h>

using namespace std;
using namespace gtsam;

/**
 * Factor to calibrate local Earth magnetic field as well as magnetometer bias
 * This version uses model measured bM = bRn * nM + bias
 * and optimizes for both nM and the bias.
 * Issue with it: expresses nM in units of magnetometer
 */
class MagFactor1: public NoiseModelFactor2<LieVector, LieVector> {

  Vector3 measured_; /** The measured magnetometer values */
  Matrix3 bRn_; /** The assumed known rotation from nav to body */

public:

  /** Constructor */
  MagFactor1(Key key1, Key key2, const Vector3& measured, const Rot3& nRb,
      const SharedNoiseModel& model) :
      NoiseModelFactor2<LieVector, LieVector>(model, key1, key2), //
      measured_(measured), bRn_(nRb.transpose()) {
  }

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new MagFactor1(*this)));
  }

  /**
   * @brief vector of errors
   * @param nM (unknown) local earth magnetic field vector, in nav frame
   * @param bias (unknown) 3D bias
   */
  Vector evaluateError(const LieVector& nM, const LieVector& bias,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    // measured bM = nRb’ * nM + b, where b is unknown bias
    Vector3 hx = bRn_ * nM + bias;
    if (H1)
      *H1 = bRn_;
    if (H2)
      *H2 = eye(3);
    return hx - measured_;
  }
};

/**
 * Factor to calibrate local Earth magnetic field as well as magnetometer bias
 * This version uses model measured bM = scale * bRn * direction + bias
 * and optimizes for both scale, direction, and the bias.
 */
class MagFactor2: public NoiseModelFactor3<LieScalar, Sphere2, LieVector> {

  Vector3 measured_; /** The measured magnetometer values */
  Rot3 bRn_; /** The assumed known rotation from nav to body */

public:

  /** Constructor */
  MagFactor2(Key key1, Key key2, Key key3, const Vector3& measured,
      const Rot3& nRb, const SharedNoiseModel& model) :
      NoiseModelFactor3<LieScalar, Sphere2, LieVector>(model, key1, key2, key3), //
      measured_(measured), bRn_(nRb.inverse()) {
  }

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new MagFactor2(*this)));
  }

  /**
   * @brief vector of errors
   * @param nM (unknown) local earth magnetic field vector, in nav frame
   * @param bias (unknown) 3D bias
   */
  Vector evaluateError(const LieScalar& scale, const Sphere2& direction,
      const LieVector& bias, boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none, boost::optional<Matrix&> H3 =
          boost::none) const {
    // measured bM = nRb’ * nM + b, where b is unknown bias
    Sphere2 rotated = bRn_.rotate(direction, boost::none, H2);
    Vector3 hx = scale * rotated.unitVector() + bias;
    if (H1)
      *H1 = rotated.unitVector();
    if (H2) // I think H2 is 2*2, but we need 3*2
    {
      Matrix H;
      rotated.unitVector(H);
      *H2 = scale * H * (*H2);
    }
    if (H3)
      *H3 = eye(3);
    return hx - measured_;
  }
};

/**
 * @file    testMagFactor.cpp
 * @brief   Unit test for MagFactor
 * @author  Frank Dellaert
 * @date   January 22, 2014
 */

//#include <gtsam/navigation/MagFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace gtsam;
using namespace GeographicLib;

// *************************************************************************
TEST( MagFactor, Constructors ) {

  using boost::none;

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
  // ...and bias
  Vector3 bias(10, -10, 50);
  // ... then we measure
  Vector3 scaled = scale * nM;
  Vector3 measured = scale * nRb.transpose() * nM + bias;

  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  Matrix expectedH1, expectedH2, expectedH3;
  Matrix H1, H2, H3;

  // MagFactor1
  MagFactor1 f1(1, 2, measured, nRb, model);
  EXPECT( assert_equal(zero(3),f1.evaluateError(scaled,bias,H1,H2),1e-5));
  EXPECT( assert_equal(numericalDerivative11<LieVector> //
      (boost::bind(&MagFactor1::evaluateError, &f1, _1, bias, none, none), scaled),//
      H1, 1e-7));
  EXPECT( assert_equal(numericalDerivative11<LieVector> //
      (boost::bind(&MagFactor1::evaluateError, &f1, scaled, _1, none, none), bias),//
      H2, 1e-7));

  // MagFactor2
  MagFactor2 f2(1, 2, 3, measured, nRb, model);
  LieScalar s(scale*nM.norm());
  Sphere2 dir(nM[0], nM[1], nM[2]);
  EXPECT(assert_equal(zero(3),f2.evaluateError(s,dir,bias,H1,H2,H3),1e-5));
  EXPECT(assert_equal(numericalDerivative11<LieScalar> //
      (boost::bind(&MagFactor2::evaluateError, &f2, _1, dir, bias, none, none, none), s),//
      H1, 1e-7));
  EXPECT(assert_equal(numericalDerivative11<Sphere2> //
      (boost::bind(&MagFactor2::evaluateError, &f2, s, _1, bias, none, none, none), dir),//
      H2, 1e-7));
  EXPECT(assert_equal(numericalDerivative11<LieVector> //
      (boost::bind(&MagFactor2::evaluateError, &f2, s, dir, _1, none, none, none), bias),//
      H3, 1e-7));
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
