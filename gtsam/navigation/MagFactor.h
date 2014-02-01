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
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/LieScalar.h>

namespace gtsam {

/**
 * Factor to estimate rotation given magnetometer reading
 * This version uses model measured bM = scale * bRn * direction + bias
 * and assumes scale, direction, and the bias are given
 */
class MagFactor: public NoiseModelFactor1<Rot2> {

  const Vector3 measured_; ///< The measured magnetometer values
  const double scale_; ///< Scale factor from direction to magnetometer readings
  const Sphere2 direction_; ///< Local magnetic field direction
  const Vector3 bias_; ///< bias

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
    if (HR) {
      HR->resize(2, 1);
      Point3 Q = q.unitVector();
      Matrix B = q.basis().transpose();
      (*HR) = Q.x() * B.col(1) - Q.y() * B.col(0);
    }
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
    if (H) {
      Matrix U;
      rotated.unitVector(U);
      *H = scale_ * U * (*H);
    }
    return hx - measured_;
  }
};

/**
 * Factor to estimate rotation given magnetometer reading
 * This version uses model measured bM = scale * bRn * direction + bias
 * and assumes scale, direction, and the bias are given
 */
class MagFactor1: public NoiseModelFactor1<Rot3> {

  const Vector3 measured_; ///< The measured magnetometer values
  const double scale_; ///< Scale factor from direction to magnetometer readings
  const Sphere2 direction_; ///< Local magnetic field direction
  const Vector3 bias_; ///< bias

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
 * and optimizes for both nM and the bias, where nM is in units defined by magnetometer
 */
class MagFactor2: public NoiseModelFactor2<LieVector, LieVector> {

  const Vector3 measured_; ///< The measured magnetometer values
  const Matrix3 bRn_; ///< The assumed known rotation from nav to body

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

  const Vector3 measured_; ///< The measured magnetometer values
  const Rot3 bRn_; ///< The assumed known rotation from nav to body

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

