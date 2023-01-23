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
 * @ingroup geometry
 */

#pragma once

#include <gtsam/geometry/Cal3.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * @brief The most common 5DOF 3D->2D calibration
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3_S2 : public Cal3 {
 public:
  enum { dimension = 5 };

  ///< shared pointer to calibration object
  using shared_ptr = std::shared_ptr<Cal3_S2>;

  /// @name Standard Constructors
  /// @{

  /// Create a default calibration that leaves coordinates unchanged
  Cal3_S2() = default;

  /// constructor from doubles
  Cal3_S2(double fx, double fy, double s, double u0, double v0)
      : Cal3(fx, fy, s, u0, v0) {}

  /// constructor from vector
  Cal3_S2(const Vector5& d) : Cal3(d) {}

  /**
   * Easy constructor, takes fov in degrees, asssumes zero skew, unit aspect
   * @param fov field of view in degrees
   * @param w image width
   * @param h image height
   */
  Cal3_S2(double fov, int w, int h) : Cal3(fov, w, h) {}

  /**
   * Convert intrinsic coordinates xy to image coordinates uv, fixed derivaitves
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*5 Jacobian wrpt Cal3 parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in image coordinates
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 5> Dcal = {},
                     OptionalJacobian<2, 2> Dp = {}) const;

  /**
   * Convert image coordinates uv to intrinsic coordinates xy
   * @param p point in image coordinates
   * @param Dcal optional 2*5 Jacobian wrpt Cal3 parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in intrinsic coordinates
   */
  Point2 calibrate(const Point2& p, OptionalJacobian<2, 5> Dcal = {},
                   OptionalJacobian<2, 2> Dp = {}) const;

  /**
   * Convert homogeneous image coordinates to intrinsic coordinates
   * @param p point in image coordinates
   * @return point in intrinsic coordinates
   */
  Vector3 calibrate(const Vector3& p) const;

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3_S2& cal);

  /// print with optional string
  void print(const std::string& s = "Cal3_S2") const override;

  /// Check if equal up to specified tolerance
  bool equals(const Cal3_S2& K, double tol = 10e-9) const;

  /// "Between", subtracts calibrations. between(p,q) == compose(inverse(p),q)
  inline Cal3_S2 between(const Cal3_S2& q,
                         OptionalJacobian<5, 5> H1 = {},
                         OptionalJacobian<5, 5> H2 = {}) const {
    if (H1) *H1 = -I_5x5;
    if (H2) *H2 = I_5x5;
    return Cal3_S2(q.fx_ - fx_, q.fy_ - fy_, q.s_ - s_, q.u0_ - u0_,
                   q.v0_ - v0_);
  }

  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline static size_t Dim() { return dimension; }

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
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Cal3_S2", boost::serialization::base_object<Cal3>(*this));
  }

  /// @}
};

template <>
struct traits<Cal3_S2> : public internal::Manifold<Cal3_S2> {};

template <>
struct traits<const Cal3_S2> : public internal::Manifold<Cal3_S2> {};

}  // \ namespace gtsam
