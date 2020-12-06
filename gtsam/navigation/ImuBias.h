/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ImuBias.h
 * @date  Feb 2, 2012
 * @author Vadim Indelman, Stephen Williams
 */

#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/VectorSpace.h>
#include <iosfwd>
#include <boost/serialization/nvp.hpp>

namespace gtsam {

/// All bias models live in the imuBias namespace
namespace imuBias {

class GTSAM_EXPORT ConstantBias {
private:
  Vector3 biasAcc_; ///< The units for stddev are σ = m/s² or m √Hz/s²
  Vector3 biasGyro_; ///< The units for stddev are σ = rad/s or rad √Hz/s

public:
  /// dimension of the variable - used to autodetect sizes
  static const size_t dimension = 6;

  ConstantBias() :
      biasAcc_(0.0, 0.0, 0.0), biasGyro_(0.0, 0.0, 0.0) {
  }

  ConstantBias(const Vector3& biasAcc, const Vector3& biasGyro) :
      biasAcc_(biasAcc), biasGyro_(biasGyro) {
  }

  explicit ConstantBias(const Vector6& v) :
      biasAcc_(v.head<3>()), biasGyro_(v.tail<3>()) {
  }

  /** return the accelerometer and gyro biases in a single vector */
  Vector6 vector() const {
    Vector6 v;
    v << biasAcc_, biasGyro_;
    return v;
  }

  /** get accelerometer bias */
  const Vector3& accelerometer() const {
    return biasAcc_;
  }

  /** get gyroscope bias */
  const Vector3& gyroscope() const {
    return biasGyro_;
  }

  /** Correct an accelerometer measurement using this bias model, and optionally compute Jacobians */
  Vector3 correctAccelerometer(const Vector3& measurement,
                               OptionalJacobian<3, 6> H1 = boost::none,
                               OptionalJacobian<3, 3> H2 = boost::none) const {
    if (H1) (*H1) << -I_3x3, Z_3x3;
    if (H2) (*H2) << I_3x3;
    return measurement - biasAcc_;
  }

  /** Correct a gyroscope measurement using this bias model, and optionally compute Jacobians */
  Vector3 correctGyroscope(const Vector3& measurement,
                           OptionalJacobian<3, 6> H1 = boost::none,
                           OptionalJacobian<3, 3> H2 = boost::none) const {
    if (H1) (*H1) << Z_3x3, -I_3x3;
    if (H2) (*H2) << I_3x3;
    return measurement - biasGyro_;
  }

  /// @}
  /// @name Testable
  /// @{

  /// ostream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const ConstantBias& bias);

  /// print with optional string
  void print(const std::string& s = "") const;

  /** equality up to tolerance */
  inline bool equals(const ConstantBias& expected, double tol = 1e-5) const {
    return equal_with_abs_tol(biasAcc_, expected.biasAcc_, tol)
        && equal_with_abs_tol(biasGyro_, expected.biasGyro_, tol);
  }

  /// @}
  /// @name Group
  /// @{

  /** identity for group operation */
  static ConstantBias identity() {
    return ConstantBias();
  }

  /** inverse */
  inline ConstantBias operator-() const {
    return ConstantBias(-biasAcc_, -biasGyro_);
  }

  /** addition of vector on right */
  ConstantBias operator+(const Vector6& v) const {
    return ConstantBias(biasAcc_ + v.head<3>(), biasGyro_ + v.tail<3>());
  }

  /** addition */
  ConstantBias operator+(const ConstantBias& b) const {
    return ConstantBias(biasAcc_ + b.biasAcc_, biasGyro_ + b.biasGyro_);
  }

  /** subtraction */
  ConstantBias operator-(const ConstantBias& b) const {
    return ConstantBias(biasAcc_ - b.biasAcc_, biasGyro_ - b.biasGyro_);
  }

  /// @}

  /// @name Deprecated
  /// @{
  ConstantBias inverse() {
    return -(*this);
  }
  ConstantBias compose(const ConstantBias& q) {
    return (*this) + q;
  }
  ConstantBias between(const ConstantBias& q) {
    return q - (*this);
  }
  Vector6 localCoordinates(const ConstantBias& q) {
    return between(q).vector();
  }
  ConstantBias retract(const Vector6& v) {
    return compose(ConstantBias(v));
  }
  static Vector6 Logmap(const ConstantBias& p) {
    return p.vector();
  }
  static ConstantBias Expmap(const Vector6& v) {
    return ConstantBias(v);
  }
  /// @}

private:

  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(biasAcc_);
    ar & BOOST_SERIALIZATION_NVP(biasGyro_);
  }


public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
  /// @}

}; // ConstantBias class
} // namespace imuBias

template<>
struct traits<imuBias::ConstantBias> : public internal::VectorSpace<
    imuBias::ConstantBias> {
};

} // namespace gtsam

