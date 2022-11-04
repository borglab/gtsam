/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Unified.h
 * @brief Unified Calibration Model, see Mei07icra for details
 * @date Mar 8, 2014
 * @author Jing Dong
 * @author Varun Agrawal
 */

/**
 * @ingroup geometry
 */

#pragma once

#include <gtsam/geometry/Cal3DS2_Base.h>

namespace gtsam {

/**
 * @brief Calibration of a omni-directional camera with mirror + lens radial
 * distortion
 * @ingroup geometry
 * \nosubgrouping
 *
 * Similar to Cal3DS2, does distortion but has additional mirror parameter xi
 * K = [ fx s u0 ; 0 fy v0 ; 0 0 1 ]
 * Pn = [ P.x / (1 + xi * \sqrt{P.x² + P.y² + 1}), P.y / (1 + xi * \sqrt{P.x² +
 * P.y² + 1})]
 * r² = Pn.x² + Pn.y²
 * \hat{pn} = (1 + k1*r² + k2*r⁴ ) pn + [ 2*k3 pn.x pn.y + k4 (r² + 2 Pn.x²) ;
 *                      k3 (rr + 2 Pn.y²) + 2*k4 pn.x pn.y  ]
 * pi = K*pn
 */
class GTSAM_EXPORT Cal3Unified : public Cal3DS2_Base {
  using This = Cal3Unified;
  using Base = Cal3DS2_Base;

 private:
  double xi_ = 0.0f;  ///< mirror parameter

 public:
  enum { dimension = 10 };

  ///< shared pointer to stereo calibration object
  using shared_ptr = boost::shared_ptr<Cal3Unified>;

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3Unified() = default;

  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1,
              double k2, double p1 = 0.0, double p2 = 0.0, double xi = 0.0)
      : Base(fx, fy, s, u0, v0, k1, k2, p1, p2), xi_(xi) {}

  ~Cal3Unified() override {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  Cal3Unified(const Vector10& v)
      : Base(v(0), v(1), v(2), v(3), v(4), v(5), v(6), v(7), v(8)), xi_(v(9)) {}

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3Unified& cal);

  /// print with optional string
  void print(const std::string& s = "") const override;

  /// assert equality up to a tolerance
  bool equals(const Cal3Unified& K, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// mirror parameter
  inline double xi() const { return xi_; }

  /// Return all parameters as a vector
  Vector10 vector() const;

  /**
   * convert intrinsic coordinates xy to image coordinates uv
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*10 Jacobian wrpt Cal3Unified parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in image coordinates
   */
  Point2 uncalibrate(const Point2& p,
                     OptionalJacobian<2, 10> Dcal = boost::none,
                     OptionalJacobian<2, 2> Dp = boost::none) const;

  /// Conver a pixel coordinate to ideal coordinate
  Point2 calibrate(const Point2& p, OptionalJacobian<2, 10> Dcal = boost::none,
                   OptionalJacobian<2, 2> Dp = boost::none) const;

  /// Convert a 3D point to normalized unit plane
  Point2 spaceToNPlane(const Point2& p) const;

  /// Convert a normalized unit plane point to 3D space
  Point2 nPlaneToSpace(const Point2& p) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Given delta vector, update calibration
  Cal3Unified retract(const Vector& d) const;

  /// Given a different calibration, calculate update to obtain it
  Vector localCoordinates(const Cal3Unified& T2) const;

  /// Return dimensions of calibration manifold object
  size_t dim() const override { return Dim(); }

  /// Return dimensions of calibration manifold object
  inline static size_t Dim() { return dimension; }

  /// @}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Cal3Unified", boost::serialization::base_object<Cal3DS2_Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(xi_);
  }
};

template <>
struct traits<Cal3Unified> : public internal::Manifold<Cal3Unified> {};

template <>
struct traits<const Cal3Unified> : public internal::Manifold<Cal3Unified> {};
}
