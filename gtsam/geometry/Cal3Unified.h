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
 */

/**
 * @addtogroup geometry
 */

#pragma once

#include <gtsam/geometry/Cal3DS2_Base.h>

namespace gtsam {

/**
 * @brief Calibration of a omni-directional camera with mirror + lens radial distortion
 * @addtogroup geometry
 * \nosubgrouping
 *
 * Similar to Cal3DS2, does distortion but has additional mirror parameter xi
 * K = [ fx s u0 ; 0 fy v0 ; 0 0 1 ]
 * Pn = [ P.x / (1 + xi * \sqrt{P.x^2 + P.y^2 + 1}), P.y / (1 + xi * \sqrt{P.x^2 + P.y^2 + 1})]
 * rr = Pn.x^2 + Pn.y^2
 * \hat{pn} = (1 + k1*rr + k2*rr^2 ) pn + [ 2*k3 pn.x pn.y + k4 (rr + 2 Pn.x^2) ;
 *                      k3 (rr + 2 Pn.y^2) + 2*k4 pn.x pn.y  ]
 * pi = K*pn
 */
class GTSAM_EXPORT Cal3Unified : public Cal3DS2_Base {

  typedef Cal3Unified This;
  typedef Cal3DS2_Base Base;

private:

  double xi_;  // mirror parameter

public:

  enum { dimension = 10 };

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3Unified() : Base(), xi_(0) {}

  Cal3Unified(double fx, double fy, double s, double u0, double v0,
      double k1, double k2, double p1 = 0.0, double p2 = 0.0, double xi = 0.0) :
        Base(fx, fy, s, u0, v0, k1, k2, p1, p2), xi_(xi) {}

  virtual ~Cal3Unified() {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  Cal3Unified(const Vector &v) ;

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  virtual void print(const std::string& s = "") const ;

  /// assert equality up to a tolerance
  bool equals(const Cal3Unified& K, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// mirror parameter
  inline double xi() const { return xi_;}

  /**
   * convert intrinsic coordinates xy to image coordinates uv
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*10 Jacobian wrpt Cal3Unified parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in image coordinates
   */
  Point2 uncalibrate(const Point2& p,
      OptionalJacobian<2,10> Dcal = boost::none,
      OptionalJacobian<2,2> Dp = boost::none) const ;

  /// Conver a pixel coordinate to ideal coordinate
  Point2 calibrate(const Point2& p, const double tol=1e-5) const;

  /// Convert a 3D point to normalized unit plane
  Point2 spaceToNPlane(const Point2& p) const;

  /// Convert a normalized unit plane point to 3D space
  Point2 nPlaneToSpace(const Point2& p) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Given delta vector, update calibration
  Cal3Unified retract(const Vector& d) const ;

  /// Given a different calibration, calculate update to obtain it
  Vector10 localCoordinates(const Cal3Unified& T2) const ;

  /// Return dimensions of calibration manifold object
  virtual size_t dim() const { return dimension ; }

  /// Return dimensions of calibration manifold object
  static size_t Dim() { return dimension; }

  /// Return all parameters as a vector
  Vector10 vector() const ;

  /// @}

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/)
  {
    ar & boost::serialization::make_nvp("Cal3Unified",
        boost::serialization::base_object<Cal3DS2_Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(xi_);
  }

};

template<>
struct traits<Cal3Unified> : public internal::Manifold<Cal3Unified> {};

template<>
struct traits<const Cal3Unified> : public internal::Manifold<Cal3Unified> {};

}

