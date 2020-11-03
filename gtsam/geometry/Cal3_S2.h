/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2.h
 * @brief  The most common 5DOF 3D->2D calibration
 * @author Frank Dellaert
 */

/**
 * @addtogroup geometry
 */

#pragma once

#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * @brief The most common 5DOF 3D->2D calibration
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3_S2 {
private:
  double fx_, fy_, s_, u0_, v0_;

public:
  enum { dimension = 5 };
  typedef boost::shared_ptr<Cal3_S2> shared_ptr; ///< shared pointer to calibration object

  /// @name Standard Constructors
  /// @{

  /// Create a default calibration that leaves coordinates unchanged
  Cal3_S2() :
      fx_(1), fy_(1), s_(0), u0_(0), v0_(0) {
  }

  /// constructor from doubles
  Cal3_S2(double fx, double fy, double s, double u0, double v0) :
      fx_(fx), fy_(fy), s_(s), u0_(u0), v0_(v0) {
  }

  /// constructor from vector
  Cal3_S2(const Vector &d) :
      fx_(d(0)), fy_(d(1)), s_(d(2)), u0_(d(3)), v0_(d(4)) {
  }

  /**
   * Easy constructor, takes fov in degrees, asssumes zero skew, unit aspect
   * @param fov field of view in degrees
   * @param w image width
   * @param h image height
   */
  Cal3_S2(double fov, int w, int h);

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// load calibration from location (default name is calibration_info.txt)
  Cal3_S2(const std::string &path);

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Cal3_S2& cal);

  /// print with optional string
  void print(const std::string& s = "Cal3_S2") const;

  /// Check if equal up to specified tolerance
  bool equals(const Cal3_S2& K, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// focal length x
  inline double fx() const {
    return fx_;
  }

  /// focal length y
  inline double fy() const {
    return fy_;
  }

  /// aspect ratio
  inline double aspectRatio() const {
    return fx_/fy_;
  }

  /// skew
  inline double skew() const {
    return s_;
  }

  /// image center in x
  inline double px() const {
    return u0_;
  }

  /// image center in y
  inline double py() const {
    return v0_;
  }

  /// return the principal point
  Point2 principalPoint() const {
    return Point2(u0_, v0_);
  }

  /// vectorized form (column-wise)
  Vector5 vector() const {
    Vector5 v;
    v << fx_, fy_, s_, u0_, v0_;
    return v;
  }

  /// return calibration matrix K
  Matrix3 K() const {
    Matrix3 K;
    K <<  fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0;
    return K;
  }

  /** @deprecated The following function has been deprecated, use K above */
  Matrix3 matrix() const {
    return K();
  }

  /// return inverted calibration matrix inv(K)
  Matrix3 matrix_inverse() const {
    const double fxy = fx_ * fy_, sv0 = s_ * v0_, fyu0 = fy_ * u0_;
    Matrix3 K_inverse;
    K_inverse << 1.0 / fx_, -s_ / fxy, (sv0 - fyu0) / fxy, 0.0,
        1.0 / fy_, -v0_ / fy_, 0.0, 0.0, 1.0;
    return K_inverse;
  }

  /**
   * convert intrinsic coordinates xy to image coordinates uv, fixed derivaitves
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*5 Jacobian wrpt Cal3_S2 parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in image coordinates
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2,5> Dcal = boost::none,
      OptionalJacobian<2,2> Dp = boost::none) const;

  /**
   * convert image coordinates uv to intrinsic coordinates xy
   * @param p point in image coordinates
   * @param Dcal optional 2*5 Jacobian wrpt Cal3_S2 parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in intrinsic coordinates
   */
  Point2 calibrate(const Point2& p, OptionalJacobian<2,5> Dcal = boost::none,
                   OptionalJacobian<2,2> Dp = boost::none) const;

  /**
   * convert homogeneous image coordinates to intrinsic coordinates
   * @param p point in image coordinates
   * @return point in intrinsic coordinates
   */
  Vector3 calibrate(const Vector3& p) const;

  /// "Between", subtracts calibrations. between(p,q) == compose(inverse(p),q)
  inline Cal3_S2 between(const Cal3_S2& q,
      OptionalJacobian<5,5> H1=boost::none,
      OptionalJacobian<5,5> H2=boost::none) const {
    if(H1) *H1 = -I_5x5;
    if(H2) *H2 =  I_5x5;
    return Cal3_S2(q.fx_-fx_, q.fy_-fy_, q.s_-s_, q.u0_-u0_, q.v0_-v0_);
  }


  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline size_t dim() const { return dimension; }

  /// return DOF, dimensionality of tangent space
  static size_t Dim() { return dimension; }

  /// Given 5-dim tangent vector, create new calibration
  inline Cal3_S2 retract(const Vector& d) const {
    return Cal3_S2(fx_ + d(0), fy_ + d(1), s_ + d(2), u0_ + d(3), v0_ + d(4));
  }

  /// Unretraction for the calibration
  Vector5 localCoordinates(const Cal3_S2& T2) const {
    return T2.vector() - vector();
  }

  /// @}
  /// @name Advanced Interface
  /// @{

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(fx_);
    ar & BOOST_SERIALIZATION_NVP(fy_);
    ar & BOOST_SERIALIZATION_NVP(s_);
    ar & BOOST_SERIALIZATION_NVP(u0_);
    ar & BOOST_SERIALIZATION_NVP(v0_);
  }

  /// @}

};

template<>
struct traits<Cal3_S2> : public internal::Manifold<Cal3_S2> {};

template<>
struct traits<const Cal3_S2> : public internal::Manifold<Cal3_S2> {};

} // \ namespace gtsam
