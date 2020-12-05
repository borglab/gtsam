/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2Stereo.h
 * @brief  The most common 5DOF 3D->2D calibration + Stereo baseline
 * @author Chris Beall
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <iosfwd>

namespace gtsam {

/**
 * @brief The most common 5DOF 3D->2D calibration, stereo version
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3_S2Stereo : public Cal3_S2 {
 private:
  double b_ = 1.0f;  ///< Stereo baseline.

 public:
  enum { dimension = 6 };

  ///< shared pointer to stereo calibration object
  using shared_ptr = boost::shared_ptr<Cal3_S2Stereo>;

  /// @name Standard Constructors
  /// @

  /// default calibration leaves coordinates unchanged
  Cal3_S2Stereo() = default;

  /// constructor from doubles
  Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b)
      : Cal3_S2(fx, fy, s, u0, v0), b_(b) {}

  /// constructor from vector
  Cal3_S2Stereo(const Vector6& d)
      : Cal3_S2(d(0), d(1), d(2), d(3), d(4)), b_(d(5)) {}

  /// easy constructor; field-of-view in degrees, assumes zero skew
  Cal3_S2Stereo(double fov, int w, int h, double b)
      : Cal3_S2(fov, w, h), b_(b) {}

  /**
   * Convert intrinsic coordinates xy to image coordinates uv, fixed derivaitves
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*6 Jacobian wrpt Cal3_S2Stereo parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in image coordinates
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 6> Dcal = boost::none,
                     OptionalJacobian<2, 2> Dp = boost::none) const;

  /**
   * Convert image coordinates uv to intrinsic coordinates xy
   * @param p point in image coordinates
   * @param Dcal optional 2*6 Jacobian wrpt Cal3_S2Stereo parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in intrinsic coordinates
   */
  Point2 calibrate(const Point2& p, OptionalJacobian<2, 6> Dcal = boost::none,
                   OptionalJacobian<2, 2> Dp = boost::none) const;

  /**
   * Convert homogeneous image coordinates to intrinsic coordinates
   * @param p point in image coordinates
   * @return point in intrinsic coordinates
   */
  Vector3 calibrate(const Vector3& p) const { return Cal3_S2::calibrate(p); }

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3_S2Stereo& cal);

  /// print with optional string
  void print(const std::string& s = "") const override;

  /// Check if equal up to specified tolerance
  bool equals(const Cal3_S2Stereo& other, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// return calibration, same for left and right
  const Cal3_S2& calibration() const { return *this; }

  /// return calibration matrix K, same for left and right
  Matrix3 K() const override { return Cal3_S2::K(); }

  /// return baseline
  inline double baseline() const { return b_; }

  /// vectorized form (column-wise)
  Vector6 vector() const {
    Vector6 v;
    v << Cal3_S2::vector(), b_;
    return v;
  }

  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline size_t dim() const override { return Dim(); }

  /// return DOF, dimensionality of tangent space
  inline static size_t Dim() { return dimension; }

  /// Given 6-dim tangent vector, create new calibration
  inline Cal3_S2Stereo retract(const Vector& d) const {
    return Cal3_S2Stereo(fx() + d(0), fy() + d(1), skew() + d(2), px() + d(3),
                         py() + d(4), b_ + d(5));
  }

  /// Unretraction for the calibration
  Vector6 localCoordinates(const Cal3_S2Stereo& T2) const {
    return T2.vector() - vector();
  }

  /// @}
  /// @name Advanced Interface
  /// @{

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Cal3_S2", boost::serialization::base_object<Cal3_S2>(*this));
    ar& BOOST_SERIALIZATION_NVP(b_);
  }
  /// @}
};

// Define GTSAM traits
template <>
struct traits<Cal3_S2Stereo> : public internal::Manifold<Cal3_S2Stereo> {};

template <>
struct traits<const Cal3_S2Stereo> : public internal::Manifold<Cal3_S2Stereo> {
};

}  // \ namespace gtsam
