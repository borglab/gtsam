/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3f.h
 * @brief  Calibration model with a single focal length and zero skew.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/Cal3.h>

namespace gtsam {

/**
 * @brief Calibration model with a single focal length and zero skew.
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3f : public Cal3 {
 public:
  constexpr static auto dimension = 1;
  using shared_ptr = std::shared_ptr<Cal3f>;

  /// @name Constructors
  /// @{

  /// Default constructor
  Cal3f() = default;

  /**
   * Constructor
   * @param f  focal length
   * @param u0 image center x-coordinate (considered a constant)
   * @param v0 image center y-coordinate (considered a constant)
   */
  Cal3f(double f, double u0, double v0);

  ~Cal3f() override = default;

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const Cal3f& cal);

  /// print with optional string
  void print(const std::string& s = "") const override;

  /// assert equality up to a tolerance
  bool equals(const Cal3f& K, double tol = 1e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// focal length
  double f() const { return fx_; }

  /// vectorized form (column-wise)
  Vector1 vector() const;

  /**
   * @brief: convert intrinsic coordinates xy to image coordinates uv
   * Version of uncalibrate with derivatives
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*1 Jacobian wrpt focal length
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in image coordinates
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 1> Dcal = {},
                     OptionalJacobian<2, 2> Dp = {}) const;

  /**
   * Convert a pixel coordinate to ideal coordinate xy
   * @param pi point in image coordinates
   * @param Dcal optional 2*1 Jacobian wrpt focal length
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in intrinsic coordinates
   */
  Point2 calibrate(const Point2& pi, OptionalJacobian<2, 1> Dcal = {},
                   OptionalJacobian<2, 2> Dp = {}) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Return DOF, dimensionality of tangent space
  size_t dim() const { return Dim(); }

  /// Return DOF, dimensionality of tangent space
  static size_t Dim() { return dimension; }

  /// Update calibration with tangent space delta
  Cal3f retract(const Vector& d) const { return Cal3f(fx_ + d(0), u0_, v0_); }

  /// Calculate local coordinates to another calibration
  Vector1 localCoordinates(const Cal3f& T2) const {
    return Vector1(T2.fx_ - fx_);
  }

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(fx_);
    ar& BOOST_SERIALIZATION_NVP(u0_);
    ar& BOOST_SERIALIZATION_NVP(v0_);
  }
#endif

  /// @}
};

template <>
struct traits<Cal3f> : public internal::Manifold<Cal3f> {};

template <>
struct traits<const Cal3f> : public internal::Manifold<Cal3f> {};

}  // namespace gtsam