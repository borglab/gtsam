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
 * @author Yong-Dian Jian
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * @brief Calibration of a camera with radial and tangential distortion.
 * @addtogroup geometry
 * \nosubgrouping
 *
 * Uses same distortionmodel as OpenCV, with
 * http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 * but using only k1,k2,p1, and p2 coefficients.
 * K = [ fx s u0 ; 0 fy v0 ; 0 0 1 ]
 * rr = Pn.x^2 + Pn.y^2
 * \hat{Pn} = \frac{(1 + k1*rr + k2*rr^2 + k3*rr^3)}{(1 + k4*rr + k5*rr^2 + k6*rr^3)} Pn + 
 *            [ 2*p1 Pn.x Pn.y + p2 (rr + 2 Pn.x^2) ;
 *                      p1 (rr + 2 Pn.y^2) + 2*p2 Pn.x Pn.y  ]
 * pi = K*Pn
 */
class GTSAM_EXPORT Cal3DS2_Base {

protected:

  double fx_, fy_, s_, u0_, v0_ ; // focal length, skew and principal point
  double k1_, k2_, k3_, k4_, k5_, k6_ ; // radial distortion parameters
  double p1_, p2_ ; // tangential distortion
  double tol_; // calibration tolerance

public:

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3DS2_Base() : fx_(1), fy_(1), s_(0), u0_(0), v0_(0), k1_(0), k2_(0),
                   k3_(0), k4_(0), k5_(0), k6_(0), p1_(0), p2_(0), tol_(1e-5) {}

  Cal3DS2_Base(double fx, double fy, double s, double u0, double v0,
      double k1, double k2, double p1 = 0.0, double p2 = 0.0,
      double k3 = 0.0, double k4 = 0.0,
      double k5 = 0.0, double k6 = 0.0, double tol=1e-5) :
  fx_(fx), fy_(fy), s_(s), u0_(u0), v0_(v0),
  k1_(k1), k2_(k2), k3_(k3), k4_(k4), k5_(k5), k6_(k6),
  p1_(p1), p2_(p2), tol_(tol) {}

  virtual ~Cal3DS2_Base() {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  Cal3DS2_Base(const Vector &v) ;

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  virtual void print(const std::string& s = "") const ;

  /// assert equality up to a tolerance
  bool equals(const Cal3DS2_Base& K, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// focal length x
  inline double fx() const { return fx_;}

  /// focal length x
  inline double fy() const { return fy_;}

  /// skew
  inline double skew() const { return s_;}

  /// image center in x
  inline double px() const { return u0_;}

  /// image center in y
  inline double py() const { return v0_;}

  /// First distortion coefficient
  inline double k1() const { return k1_;}

  /// Second distortion coefficient
  inline double k2() const { return k2_;}

  /// Third distortion coefficient
  inline double k3() const { return k3_;}

  /// Fourth distortion coefficient
  inline double k4() const { return k4_;}

  /// Fifth distortion coefficient
  inline double k5() const { return k5_;}

  /// Sixth distortion coefficient
  inline double k6() const { return k6_;}

  /// First tangential distortion coefficient
  inline double p1() const { return p1_;}

  /// Second tangential distortion coefficient
  inline double p2() const { return p2_;}

  /// return calibration matrix -- not really applicable
  Matrix3 K() const;

  /// return distortion parameter vector
  Vector8 k() const {
    Vector8 k;
    return ( k << k1_, k2_, p1_, p2_, k3_, k4_, k5_, k6_ ).finished();
  }

  /// Return all parameters as a vector
  Vector13 vector() const;

  /**
   * convert intrinsic coordinates xy to (distorted) image coordinates uv
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*13 Jacobian wrpt Cal3DS2 parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in (distorted) image coordinates
   */
  Point2 uncalibrate(const Point2& p,
       OptionalJacobian<2,13> Dcal = boost::none,
       OptionalJacobian<2,2> Dp = boost::none) const ;

  /// Convert (distorted) image coordinates uv to intrinsic coordinates xy
  Point2 calibrate(const Point2& p,
       OptionalJacobian<2,9> Dcal = boost::none,
       OptionalJacobian<2,2> Dp = boost::none) const;

  /// Derivative of uncalibrate wrpt intrinsic coordinates
  Matrix2 D2d_intrinsic(const Point2& p) const ;

  /// Derivative of uncalibrate wrpt the calibration parameters
  Eigen::Matrix<double, 2, 13> D2d_calibration(const Point2& p) const ;

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
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/)
  {
    ar & BOOST_SERIALIZATION_NVP(fx_);
    ar & BOOST_SERIALIZATION_NVP(fy_);
    ar & BOOST_SERIALIZATION_NVP(s_);
    ar & BOOST_SERIALIZATION_NVP(u0_);
    ar & BOOST_SERIALIZATION_NVP(v0_);
    ar & BOOST_SERIALIZATION_NVP(k1_);
    ar & BOOST_SERIALIZATION_NVP(k2_);
    ar & BOOST_SERIALIZATION_NVP(p1_);
    ar & BOOST_SERIALIZATION_NVP(p2_);
  }

  /// @}

};

}

