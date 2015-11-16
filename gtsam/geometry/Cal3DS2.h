/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2.h
 * @brief Calibration of a camera with radial distortion, calculations in base class Cal3DS2_Base
 * @date Feb 28, 2010
 * @author ydjian
 */

#pragma once

#include <gtsam/geometry/Cal3DS2_Base.h>

namespace gtsam {

/**
 * @brief Calibration of a camera with radial distortion
 * @addtogroup geometry
 * \nosubgrouping
 *
 * Uses same distortionmodel as OpenCV, with
 * http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 * but using only k1,k2,p1, and p2 coefficients.
 * K = [ fx s u0 ; 0 fy v0 ; 0 0 1 ]
 * rr = Pn.x^2 + Pn.y^2
 * \hat{pn} = (1 + k1*rr + k2*rr^2 ) pn + [ 2*k3 pn.x pn.y + k4 (rr + 2 Pn.x^2) ;
 *                      k3 (rr + 2 Pn.y^2) + 2*k4 pn.x pn.y  ]
 * pi = K*pn
 */
class GTSAM_EXPORT Cal3DS2 : public Cal3DS2_Base {

  typedef Cal3DS2_Base Base;

public:

  enum { dimension = 9 };

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3DS2() : Base() {}

  Cal3DS2(double fx, double fy, double s, double u0, double v0,
      double k1, double k2, double p1 = 0.0, double p2 = 0.0) :
        Base(fx, fy, s, u0, v0, k1, k2, p1, p2) {}

  virtual ~Cal3DS2() {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  Cal3DS2(const Vector &v) : Base(v) {}

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  virtual void print(const std::string& s = "") const ;

  /// assert equality up to a tolerance
  bool equals(const Cal3DS2& K, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Given delta vector, update calibration
  Cal3DS2 retract(const Vector& d) const ;

  /// Given a different calibration, calculate update to obtain it
  Vector localCoordinates(const Cal3DS2& T2) const ;

  /// Return dimensions of calibration manifold object
  virtual size_t dim() const { return 9 ; } //TODO: make a final dimension variable (also, usually size_t in other classes e.g. Pose2)

  /// Return dimensions of calibration manifold object
  static size_t Dim() { return 9; }  //TODO: make a final dimension variable

  /// @}
  /// @name Clone
  /// @{

  /// @return a deep copy of this object
  virtual boost::shared_ptr<Base> clone() const {
    return boost::shared_ptr<Base>(new Cal3DS2(*this));
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
    ar & boost::serialization::make_nvp("Cal3DS2",
        boost::serialization::base_object<Cal3DS2_Base>(*this));
  }

  /// @}

};

template<>
struct traits<Cal3DS2> : public internal::Manifold<Cal3DS2> {};

template<>
struct traits<const Cal3DS2> : public internal::Manifold<Cal3DS2> {};

}

