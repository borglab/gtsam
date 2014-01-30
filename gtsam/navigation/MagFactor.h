/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    MagFactor.h
 * @brief   Factors involving magnetometers
 * @author  Frank Dellaert
 * @date   January 29, 2014
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/LieScalar.h>

namespace gtsam {

/**
 * Factor to estimate rotation given magnetometer reading
 * This version uses model measured bM = scale * bRn * direction + bias
 * and assumes scale, direction, and the bias are given
 */
class MagFactor1: public NoiseModelFactor1<Rot3> {

  const Vector3 measured_; /** The measured magnetometer values */
  const double scale_;
  const Sphere2 direction_;
  const Vector3 bias_;

public:

  /** Constructor */
  MagFactor1(Key key, const Vector3& measured, const LieScalar& scale,
      const Sphere2& direction, const LieVector& bias,
      const SharedNoiseModel& model) :
      NoiseModelFactor1<Rot3>(model, key), //
      measured_(measured), scale_(scale), direction_(direction), bias_(bias) {
  }

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new MagFactor1(*this)));
  }

  /**
   * @brief vector of errors
   */
  Vector evaluateError(const Rot3& nRb,
      boost::optional<Matrix&> H = boost::none) const {
    // measured bM = nRb’ * nM + b, where b is unknown bias
    Sphere2 rotated = nRb.unrotate(direction_, H);
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

/**
 * Factor to calibrate local Earth magnetic field as well as magnetometer bias
 * This version uses model measured bM = bRn * nM + bias
 * and optimizes for both nM and the bias.
 * Issue with it: expresses nM in units of magnetometer
 */
class MagFactor2: public NoiseModelFactor2<LieVector, LieVector> {

  Vector3 measured_; /** The measured magnetometer values */
  Matrix3 bRn_; /** The assumed known rotation from nav to body */

public:

  /** Constructor */
  MagFactor2(Key key1, Key key2, const Vector3& measured, const Rot3& nRb,
      const SharedNoiseModel& model) :
      NoiseModelFactor2<LieVector, LieVector>(model, key1, key2), //
      measured_(measured), bRn_(nRb.transpose()) {
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
class MagFactor3: public NoiseModelFactor3<LieScalar, Sphere2, LieVector> {

  Vector3 measured_; /** The measured magnetometer values */
  Rot3 bRn_; /** The assumed known rotation from nav to body */

public:

  /** Constructor */
  MagFactor3(Key key1, Key key2, Key key3, const Vector3& measured,
      const Rot3& nRb, const SharedNoiseModel& model) :
      NoiseModelFactor3<LieScalar, Sphere2, LieVector>(model, key1, key2, key3), //
      measured_(measured), bRn_(nRb.inverse()) {
  }

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new MagFactor3(*this)));
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

}

