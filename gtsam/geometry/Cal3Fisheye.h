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
 */

#pragma once

#include <gtsam/geometry/Point2.h>

#include <string>

namespace gtsam {

/**
 * @brief Calibration of a fisheye camera
 * @addtogroup geometry
 * \nosubgrouping
 *
 * Uses same distortionmodel as OpenCV, with
 * https://docs.opencv.org/master/db/d58/group__calib3d__fisheye.html
 * 3D point in camera frame
 *   p = (x, y, z)
 * Intrinsic coordinates:
 *   [x_i;y_i] = [x/z; y/z]
 * Distorted coordinates:
 *   r^2 = (x_i)^2 + (y_i)^2
 *   th = atan(r)
 *   th_d = th(1 + k1*th^2 + k2*th^4 + k3*th^6 + k4*th^8)
 *   [x_d; y_d] = (th_d / r)*[x_i; y_i]
 * Pixel coordinates:
 *   K = [fx s u0; 0 fy v0 ;0 0 1]
 *   [u; v; 1] = K*[x_d; y_d; 1]
 */
class GTSAM_EXPORT Cal3Fisheye {
 private:
  double fx_, fy_, s_, u0_, v0_;  // focal length, skew and principal point
  double k1_, k2_, k3_, k4_;      // fisheye distortion coefficients

 public:
  enum { dimension = 9 };
  typedef boost::shared_ptr<Cal3Fisheye>
      shared_ptr;  ///< shared pointer to fisheye calibration object

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3Fisheye()
      : fx_(1), fy_(1), s_(0), u0_(0), v0_(0), k1_(0), k2_(0), k3_(0), k4_(0) {}

  Cal3Fisheye(const double fx, const double fy, const double s, const double u0,
              const double v0, const double k1, const double k2,
              const double k3, const double k4)
      : fx_(fx),
        fy_(fy),
        s_(s),
        u0_(u0),
        v0_(v0),
        k1_(k1),
        k2_(k2),
        k3_(k3),
        k4_(k4) {}

  virtual ~Cal3Fisheye() {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit Cal3Fisheye(const Vector9& v);

  /// @}
  /// @name Standard Interface
  /// @{

  /// focal length x
  inline double fx() const { return fx_; }

  /// focal length x
  inline double fy() const { return fy_; }

  /// skew
  inline double skew() const { return s_; }

  /// image center in x
  inline double px() const { return u0_; }

  /// image center in y
  inline double py() const { return v0_; }

  /// First distortion coefficient
  inline double k1() const { return k1_; }

  /// Second distortion coefficient
  inline double k2() const { return k2_; }

  /// First tangential distortion coefficient
  inline double k3() const { return k3_; }

  /// Second tangential distortion coefficient
  inline double k4() const { return k4_; }

  /// return calibration matrix
  Matrix3 K() const;

  /// return distortion parameter vector
  Vector4 k() const { return Vector4(k1_, k2_, k3_, k4_); }

  /// Return all parameters as a vector
  Vector9 vector() const;

  /// Helper function that calculates atan(r)/r
  static double Scaling(double r);

  /**
   * @brief convert intrinsic coordinates [x_i; y_i] to (distorted) image
   * coordinates [u; v]
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*9 Jacobian wrpt intrinsic parameters (fx, fy, ...,
   * k4)
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates (xi, yi)
   * @return point in (distorted) image coordinates
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 9> Dcal = boost::none,
                     OptionalJacobian<2, 2> Dp = boost::none) const;

  /// Convert (distorted) image coordinates [u;v] to intrinsic coordinates [x_i,
  /// y_i]
  Point2 calibrate(const Point2& p, const double tol = 1e-5) const;

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  virtual void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const Cal3Fisheye& K, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Given delta vector, update calibration
  Cal3Fisheye retract(const Vector& d) const;

  /// Given a different calibration, calculate update to obtain it
  Vector localCoordinates(const Cal3Fisheye& T2) const;

  /// Return dimensions of calibration manifold object
  virtual size_t dim() const { return 9; }

  /// Return dimensions of calibration manifold object
  static size_t Dim() { return 9; }

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
    ar& BOOST_SERIALIZATION_NVP(fx_);
    ar& BOOST_SERIALIZATION_NVP(fy_);
    ar& BOOST_SERIALIZATION_NVP(s_);
    ar& BOOST_SERIALIZATION_NVP(u0_);
    ar& BOOST_SERIALIZATION_NVP(v0_);
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
