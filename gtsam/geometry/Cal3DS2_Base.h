/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2.h
 * @brief Calibration of a camera with radial distortion
 * @date Feb 28, 2010
 * @author ydjian
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/geometry/Cal3.h>
#include <gtsam/geometry/Point2.h>
#include <boost/shared_ptr.hpp>

namespace gtsam {

/**
 * @brief Calibration of a camera with radial distortion
 * @ingroup geometry
 * \nosubgrouping
 *
 * Uses same distortionmodel as OpenCV, with
 * http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 * but using only k1,k2,p1, and p2 coefficients.
 * K = [ fx s u0 ; 0 fy v0 ; 0 0 1 ]
 * r² = P.x² + P.y²
 * P̂ = (1 + k1*r² + k2*r⁴) P + [ (2*p1 P.x P.y) + p2 (r² + 2 Pn.x²)
 *                            p1 (r² + 2 Pn.y²) + (2*p2 Pn.x Pn.y) ]
 * pi = K*P̂
 */
class GTSAM_EXPORT Cal3DS2_Base : public Cal3 {
 protected:
  double k1_ = 0.0f, k2_ = 0.0f;  ///< radial 2nd-order and 4th-order
  double p1_ = 0.0f, p2_ = 0.0f;  ///< tangential distortion
  double tol_ = 1e-5;             ///< tolerance value when calibrating

 public:
  enum { dimension = 9 };

  ///< shared pointer to stereo calibration object
  using shared_ptr = boost::shared_ptr<Cal3DS2_Base>;

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3DS2_Base() = default;

  Cal3DS2_Base(double fx, double fy, double s, double u0, double v0, double k1,
               double k2, double p1 = 0.0, double p2 = 0.0, double tol = 1e-5)
      : Cal3(fx, fy, s, u0, v0),
        k1_(k1),
        k2_(k2),
        p1_(p1),
        p2_(p2),
        tol_(tol) {}

  ~Cal3DS2_Base() override {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  Cal3DS2_Base(const Vector9& v)
      : Cal3(v(0), v(1), v(2), v(3), v(4)),
        k1_(v(5)),
        k2_(v(6)),
        p1_(v(7)),
        p2_(v(8)) {}

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3DS2_Base& cal);

  /// print with optional string
  void print(const std::string& s = "") const override;

  /// assert equality up to a tolerance
  bool equals(const Cal3DS2_Base& K, double tol = 1e-8) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// First distortion coefficient
  inline double k1() const { return k1_; }

  /// Second distortion coefficient
  inline double k2() const { return k2_; }

  /// First tangential distortion coefficient
  inline double p1() const { return p1_; }

  /// Second tangential distortion coefficient
  inline double p2() const { return p2_; }

  /// return distortion parameter vector
  Vector4 k() const { return Vector4(k1_, k2_, p1_, p2_); }

  /// Return all parameters as a vector
  Vector9 vector() const;

  /**
   * convert intrinsic coordinates xy to (distorted) image coordinates uv
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*9 Jacobian wrpt Cal3DS2 parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in (distorted) image coordinates
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 9> Dcal = boost::none,
                     OptionalJacobian<2, 2> Dp = boost::none) const;

  /// Convert (distorted) image coordinates uv to intrinsic coordinates xy
  Point2 calibrate(const Point2& p, OptionalJacobian<2, 9> Dcal = boost::none,
                   OptionalJacobian<2, 2> Dp = boost::none) const;

  /// Derivative of uncalibrate wrpt intrinsic coordinates
  Matrix2 D2d_intrinsic(const Point2& p) const;

  /// Derivative of uncalibrate wrpt the calibration parameters
  Matrix29 D2d_calibration(const Point2& p) const;

  /// return DOF, dimensionality of tangent space
  size_t dim() const override { return Dim(); }

  /// return DOF, dimensionality of tangent space
  inline static size_t Dim() { return dimension; }

  /// @}
  /// @name Clone
  /// @{

  /// @return a deep copy of this object
  virtual boost::shared_ptr<Cal3DS2_Base> clone() const {
    return boost::shared_ptr<Cal3DS2_Base>(new Cal3DS2_Base(*this));
  }

  /// @}

 private:
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Cal3DS2_Base", boost::serialization::base_object<Cal3>(*this));
    ar& BOOST_SERIALIZATION_NVP(k1_);
    ar& BOOST_SERIALIZATION_NVP(k2_);
    ar& BOOST_SERIALIZATION_NVP(p1_);
    ar& BOOST_SERIALIZATION_NVP(p2_);
    ar& BOOST_SERIALIZATION_NVP(tol_);
  }

  /// @}
};
}
