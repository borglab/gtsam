/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Fisheye.h
 * @brief Calibration of a fisheye camera
 * @date Apr 8, 2020
 * @author ghaggin
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/geometry/Cal3.h>
#include <gtsam/geometry/Point2.h>

#include <boost/shared_ptr.hpp>
#include <string>

namespace gtsam {

/**
 * @brief Calibration of a fisheye camera
 * @ingroup geometry
 * \nosubgrouping
 *
 * Uses same distortionmodel as OpenCV, with
 * https://docs.opencv.org/master/db/d58/group__calib3d__fisheye.html
 * 3D point in camera frame
 *   p = (x, y, z)
 * Intrinsic coordinates:
 *   [x_i;y_i] = [x/z; y/z]
 * Distorted coordinates:
 *   r² = (x_i)² + (y_i)²
 *   th = atan(r)
 *   th_d = th(1 + k1*th² + k2*th⁴ + k3*th⁶ + k4*th⁸)
 *   [x_d; y_d] = (th_d / r)*[x_i; y_i]
 * Pixel coordinates:
 *   K = [fx s u0; 0 fy v0 ;0 0 1]
 *   [u; v; 1] = K*[x_d; y_d; 1]
 */
class GTSAM_EXPORT Cal3Fisheye : public Cal3 {
 private:
  double k1_ = 0.0f, k2_ = 0.0f;  ///< fisheye distortion coefficients
  double k3_ = 0.0f, k4_ = 0.0f;  ///< fisheye distortion coefficients
  double tol_ = 1e-5;             ///< tolerance value when calibrating

 public:
  enum { dimension = 9 };
  ///< shared pointer to fisheye calibration object
  using shared_ptr = boost::shared_ptr<Cal3Fisheye>;

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3Fisheye() = default;

  Cal3Fisheye(const double fx, const double fy, const double s, const double u0,
              const double v0, const double k1, const double k2,
              const double k3, const double k4, double tol = 1e-5)
      : Cal3(fx, fy, s, u0, v0),
        k1_(k1),
        k2_(k2),
        k3_(k3),
        k4_(k4),
        tol_(tol) {}

  ~Cal3Fisheye() override {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit Cal3Fisheye(const Vector9& v)
      : Cal3(v(0), v(1), v(2), v(3), v(4)),
        k1_(v(5)),
        k2_(v(6)),
        k3_(v(7)),
        k4_(v(8)) {}

  /// @}
  /// @name Standard Interface
  /// @{

  /// First distortion coefficient
  inline double k1() const { return k1_; }

  /// Second distortion coefficient
  inline double k2() const { return k2_; }

  /// First tangential distortion coefficient
  inline double k3() const { return k3_; }

  /// Second tangential distortion coefficient
  inline double k4() const { return k4_; }

  /// return distortion parameter vector
  Vector4 k() const { return Vector4(k1_, k2_, k3_, k4_); }

  /// Return all parameters as a vector
  Vector9 vector() const;

  /**
   * @brief convert intrinsic coordinates [x_i; y_i] to (distorted) image
   * coordinates [u; v]
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*9 Jacobian wrpt intrinsic parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates (xi, yi)
   * @return point in (distorted) image coordinates
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 9> Dcal = boost::none,
                     OptionalJacobian<2, 2> Dp = boost::none) const;

  /**
   * Convert (distorted) image coordinates [u;v] to intrinsic coordinates [x_i,
   * y_i]
   * @param p point in image coordinates
   * @param Dcal optional 2*9 Jacobian wrpt intrinsic parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates (xi, yi)
   * @return point in intrinsic coordinates
   */
  Point2 calibrate(const Point2& p, OptionalJacobian<2, 9> Dcal = boost::none,
                   OptionalJacobian<2, 2> Dp = boost::none) const;

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3Fisheye& cal);

  /// print with optional string
  void print(const std::string& s = "") const override;

  /// assert equality up to a tolerance
  bool equals(const Cal3Fisheye& K, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Return dimensions of calibration manifold object
  size_t dim() const override { return Dim(); }

  /// Return dimensions of calibration manifold object
  inline static size_t Dim() { return dimension; }

  /// Given delta vector, update calibration
  inline Cal3Fisheye retract(const Vector& d) const {
    return Cal3Fisheye(vector() + d);
  }

  /// Given a different calibration, calculate update to obtain it
  Vector localCoordinates(const Cal3Fisheye& T2) const {
    return T2.vector() - vector();
  }

  /// @}
  /// @name Clone
  /// @{

  /// @return a deep copy of this object
  virtual boost::shared_ptr<Cal3Fisheye> clone() const {
    return boost::shared_ptr<Cal3Fisheye>(new Cal3Fisheye(*this));
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
        "Cal3Fisheye", boost::serialization::base_object<Cal3>(*this));
    ar& BOOST_SERIALIZATION_NVP(k1_);
    ar& BOOST_SERIALIZATION_NVP(k2_);
    ar& BOOST_SERIALIZATION_NVP(k3_);
    ar& BOOST_SERIALIZATION_NVP(k4_);
  }

  /// @}
};

template <>
struct traits<Cal3Fisheye> : public internal::Manifold<Cal3Fisheye> {};

template <>
struct traits<const Cal3Fisheye> : public internal::Manifold<Cal3Fisheye> {};

}  // namespace gtsam
