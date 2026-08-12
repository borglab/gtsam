/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Bundler.h
 * @brief Calibration used by Bundler
 * @date Sep 25, 2010
 * @author Yong Dian Jian
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/geometry/Cal3f.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * @brief Calibration used by Bundler
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3Bundler : public Cal3f {
 private:
  double k1_ = 0.0, k2_ = 0.0;  ///< radial distortion coefficients
  double tol_ = 1e-5;           ///< tolerance value when calibrating

  // Note: u0 and v0 are constants and not optimized.

 public:
  constexpr static auto dimension = 3;
  using shared_ptr = std::shared_ptr<Cal3Bundler>;

  /// @name Constructors
  /// @{

  /// Default constructor
  Cal3Bundler() = default;

  /**
   *  Constructor
   *  @param f focal length
   *  @param k1 first radial distortion coefficient (quadratic)
   *  @param k2 second radial distortion coefficient (quartic)
   *  @param u0 optional image center (default 0), considered a constant
   *  @param v0 optional image center (default 0), considered a constant
   *  @param tol optional calibration tolerance value
   */
  Cal3Bundler(double f, double k1, double k2, double u0 = 0, double v0 = 0,
              double tol = 1e-5)
      : Cal3f(f, u0, v0), k1_(k1), k2_(k2), tol_(tol) {}

  ~Cal3Bundler() override = default;

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3Bundler& cal);

  /// print with optional string
  void print(const std::string& s = "") const override;

  /// assert equality up to a tolerance
  bool equals(const Cal3Bundler& K, double tol = 1e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// distortion parameter k1
  double k1() const { return k1_; }

  /// distortion parameter k2
  double k2() const { return k2_; }

  Matrix3 K() const override;  ///< Standard 3*3 calibration matrix
  Vector4 k() const;  ///< Radial distortion parameters (4 of them, 2 0)

  Vector3 vector() const;

  /**
   * @brief: convert intrinsic coordinates xy to image coordinates uv
   * Version of uncalibrate with derivatives
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*3 Jacobian wrpt Cal3Bundler parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in image coordinates
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 3> Dcal = {},
                     OptionalJacobian<2, 2> Dp = {}) const;

  /**
   * Convert a pixel coordinate to ideal coordinate xy
   * @param pi point in image coordinates
   * @param Dcal optional 2*3 Jacobian wrpt Cal3Bundler parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in intrinsic coordinates
   */
  Point2 calibrate(const Point2& pi, OptionalJacobian<2, 3> Dcal = {},
                   OptionalJacobian<2, 2> Dp = {}) const;

  /// @deprecated might be removed in next release, use uncalibrate
  Matrix2 D2d_intrinsic(const Point2& p) const;

  /// @deprecated might be removed in next release, use uncalibrate
  Matrix23 D2d_calibration(const Point2& p) const;

  /// @deprecated might be removed in next release, use uncalibrate
  Matrix25 D2d_intrinsic_calibration(const Point2& p) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Return DOF, dimensionality of tangent space
  size_t dim() const { return Dim(); }

  /// Return DOF, dimensionality of tangent space
  static size_t Dim() { return dimension; }

  /// Update calibration with tangent space delta
  Cal3Bundler retract(const Vector& d) const {
    return Cal3Bundler(fx_ + d(0), k1_ + d(1), k2_ + d(2), u0_, v0_);
  }

  /// Calculate local coordinates to another calibration
  Vector3 localCoordinates(const Cal3Bundler& T2) const {
    return T2.vector() - vector();
  }

 private:
  /// @}
  /// @name Advanced Interface
  /// @{

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Cal3Bundler", boost::serialization::base_object<Cal3>(*this));
    ar& BOOST_SERIALIZATION_NVP(k1_);
    ar& BOOST_SERIALIZATION_NVP(k2_);
    ar& BOOST_SERIALIZATION_NVP(tol_);
  }
#endif

  /// @}
};

template <>
struct traits<Cal3Bundler> : public internal::Manifold<Cal3Bundler> {};

template <>
struct traits<const Cal3Bundler> : public internal::Manifold<Cal3Bundler> {};

}  // namespace gtsam
