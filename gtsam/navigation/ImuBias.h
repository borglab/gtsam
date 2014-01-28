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

#include <boost/serialization/nvp.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Pose3.h>

/*
 * NOTES:
 * - Earth-rate correction:
 *     + Currently the user should supply R_ECEF_to_G, which is the rotation from ECEF to Local-Level system (NED or ENU as defined by the user).
 *     + R_ECEF_to_G can be calculated by approximated values of latitude and longitude of the system.
 *     + A relatively small distance is traveled w.r.t. to initial pose is assumed, since R_ECEF_to_G is constant.
 *        Otherwise, R_ECEF_to_G should be updated each time using the current lat-lon.
 *
 *  - Currently, an empty constructed is not enabled so that the user is forced to specify R_ECEF_to_G.
 */

namespace gtsam {

/// All bias models live in the imuBias namespace
namespace imuBias {

  class ConstantBias : public DerivedValue<ConstantBias> {
  private:
    Vector3 biasAcc_;
    Vector3 biasGyro_;

  public:
    /// dimension of the variable - used to autodetect sizes
    static const size_t dimension = 6;

    ConstantBias():
      biasAcc_(0.0, 0.0, 0.0),  biasGyro_(0.0, 0.0, 0.0) {
    }

    ConstantBias(const Vector3& biasAcc, const Vector3& biasGyro):
      biasAcc_(biasAcc), biasGyro_(biasGyro) {
    }

    /** return the accelerometer and gyro biases in a single vector */
    Vector vector() const {
      Vector v(6);
      v << biasAcc_, biasGyro_;
      return v;
    }

    /** get accelerometer bias */
    const Vector3& accelerometer() const { return biasAcc_; }

    /** get gyroscope bias */
    const Vector3& gyroscope() const { return biasGyro_; }

    /** Correct an accelerometer measurement using this bias model, and optionally compute Jacobians */
    Vector correctAccelerometer(const Vector3& measurement, boost::optional<Matrix&> H=boost::none) const {
      if (H){
        H->resize(3,6);
        (*H) << -Matrix3::Identity(), Matrix3::Zero();
      }
      return measurement - biasAcc_;
    }

    /** Correct a gyroscope measurement using this bias model, and optionally compute Jacobians */
    Vector correctGyroscope(const Vector3& measurement, boost::optional<Matrix&> H=boost::none) const {
      if (H){
        H->resize(3,6);
        (*H) << Matrix3::Zero(), -Matrix3::Identity();
      }
      return measurement - biasGyro_;
    }

//    // H1: Jacobian w.r.t. IMUBias
//    // H2: Jacobian w.r.t. pose
//    Vector CorrectGyroWithEarthRotRate(Vector measurement, const Pose3& pose, const Vector& w_earth_rate_G,
//        boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const {
//
//      Matrix R_G_to_I( pose.rotation().matrix().transpose() );
//      Vector w_earth_rate_I = R_G_to_I * w_earth_rate_G;
//
//      if (H1){
//        Matrix zeros3_3(zeros(3,3));
//        Matrix m_eye3(-eye(3));
//
//        *H1 = collect(2, &zeros3_3, &m_eye3);
//      }
//
//      if (H2){
//        Matrix zeros3_3(zeros(3,3));
//        Matrix H = -skewSymmetric(w_earth_rate_I);
//
//        *H2 = collect(2, &H, &zeros3_3);
//      }
//
//      //TODO: Make sure H2 is correct.
//
//      return measurement - biasGyro_ - w_earth_rate_I;
//
////      Vector bias_gyro_temp((Vector(3) << -bias_gyro_(0), bias_gyro_(1), bias_gyro_(2)));
////      return measurement - bias_gyro_temp - R_G_to_I * w_earth_rate_G;
//    }

    /// @}
    /// @name Testable
    /// @{

    /// print with optional string
    void print(const std::string& s = "") const {
      // explicit printing for now.
      std::cout << s + ".biasAcc [" << biasAcc_.transpose() << "]" << std::endl;
      std::cout << s + ".biasGyro [" << biasGyro_.transpose() << "]" << std::endl;
    }

    /** equality up to tolerance */
    inline bool equals(const ConstantBias& expected, double tol=1e-5) const {
      return equal_with_abs_tol(biasAcc_, expected.biasAcc_, tol)
          && equal_with_abs_tol(biasGyro_, expected.biasGyro_, tol);
    }

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    inline static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space
    inline size_t dim() const { return dimension; }

    /** Update the LieVector with a tangent space update */
    inline ConstantBias retract(const Vector& v) const { return ConstantBias(biasAcc_ + v.head(3), biasGyro_ + v.tail(3)); }

    /** @return the local coordinates of another object */
    inline Vector localCoordinates(const ConstantBias& b) const { return b.vector() - vector(); }

    /// @}
    /// @name Group
    /// @{

    /** identity for group operation */
    static ConstantBias identity() { return ConstantBias(); }

    /** invert the object and yield a new one */
    inline ConstantBias inverse(boost::optional<Matrix&> H=boost::none) const {
      if(H) *H = -gtsam::Matrix::Identity(6,6);

      return ConstantBias(-biasAcc_, -biasGyro_);
    }

    ConstantBias compose(const ConstantBias& b,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const {
      if(H1) *H1 = gtsam::Matrix::Identity(6,6);
      if(H2) *H2 = gtsam::Matrix::Identity(6,6);

      return ConstantBias(biasAcc_ + b.biasAcc_, biasGyro_ + b.biasGyro_);
    }

    /** between operation */
    ConstantBias between(const ConstantBias& b,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const {
      if(H1) *H1 = -gtsam::Matrix::Identity(6,6);
      if(H2) *H2 = gtsam::Matrix::Identity(6,6);

      return ConstantBias(b.biasAcc_ - biasAcc_, b.biasGyro_ - biasGyro_);
    }

    /// @}
    /// @name Lie Group
    /// @{

    /** Expmap around identity */
    static inline ConstantBias Expmap(const Vector& v) { return ConstantBias(v.head(3), v.tail(3)); }

    /** Logmap around identity - just returns with default cast back */
    static inline Vector Logmap(const ConstantBias& b) { return b.vector(); }

    /// @}

  private:

    /// @name Advanced Interface
    /// @{

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version)
    {
      ar & boost::serialization::make_nvp("imuBias::ConstantBias",
          boost::serialization::base_object<Value>(*this));
      ar & BOOST_SERIALIZATION_NVP(biasAcc_);
      ar & BOOST_SERIALIZATION_NVP(biasGyro_);
    }

    /// @}

  }; // ConstantBias class


} // namespace ImuBias

} // namespace gtsam


