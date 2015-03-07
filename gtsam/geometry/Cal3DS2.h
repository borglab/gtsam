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
 */

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * @brief Calibration of a camera with radial distortion
 * @addtogroup geometry
 * \nosubgrouping
 */
class Cal3DS2 : public DerivedValue<Cal3DS2> {

private:

	double fx_, fy_, s_, u0_, v0_ ; // focal length, skew and principal point
	double k1_, k2_ ; // radial 2nd-order and 4th-order
	double k3_, k4_ ; // tangential distortion

	// K = [ fx s u0 ; 0 fy v0 ; 0 0 1 ]
	// r = Pn.x^2 + Pn.y^2
	// \hat{pn} = (1 + k1*r + k2*r^2 ) pn + [ 2*k3 pn.x pn.y + k4 (r + 2 Pn.x^2) ;
	//										  k3 (r + 2 Pn.y^2) + 2*k4 pn.x pn.y  ]
	// pi = K*pn

public:
	Matrix K() const ;
	Eigen::Vector4d k() const { return Eigen::Vector4d(k1_, k2_, k3_, k4_); }
	Vector vector() const ;

  /// @name Standard Constructors
  /// @{

	/// Default Constructor with only unit focal length
	Cal3DS2() : fx_(1), fy_(1), s_(0), u0_(0), v0_(0), k1_(0), k2_(0), k3_(0), k4_(0) {}

	Cal3DS2(double fx, double fy, double s, double u0, double v0,
			double k1, double k2, double k3 = 0.0, double k4 = 0.0) :
	fx_(fx), fy_(fy), s_(s), u0_(u0), v0_(v0), k1_(k1), k2_(k2), k3_(k3), k4_(k4) {}

  /// @}
  /// @name Advanced Constructors
  /// @{

	Cal3DS2(const Vector &v) ;

	/// @}
	/// @name Testable
	/// @{

	/// print with optional string
	void print(const std::string& s = "") const ;

	/// assert equality up to a tolerance
	bool equals(const Cal3DS2& K, double tol = 10e-9) const;

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

	/// Convert ideal point p (in intrinsic coordinates) into pixel coordinates
	Point2 uncalibrate(const Point2& p,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const ;

	/// Conver a pixel coordinate to ideal coordinate
  Point2 calibrate(const Point2& p, const double tol=1e-5) const;

	/// Derivative of uncalibrate wrpt intrinsic coordinates
	Matrix D2d_intrinsic(const Point2& p) const ;

  /// Derivative of uncalibrate wrpt the calibration parameters
	Matrix D2d_calibration(const Point2& p) const ;

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
	static size_t Dim() { return 9; }	//TODO: make a final dimension variable

private:

  /// @}
  /// @name Advanced Interface
  /// @{

	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::make_nvp("Cal3DS2",
				boost::serialization::base_object<Value>(*this));
		ar & BOOST_SERIALIZATION_NVP(fx_);
		ar & BOOST_SERIALIZATION_NVP(fy_);
		ar & BOOST_SERIALIZATION_NVP(s_);
		ar & BOOST_SERIALIZATION_NVP(u0_);
		ar & BOOST_SERIALIZATION_NVP(v0_);
		ar & BOOST_SERIALIZATION_NVP(k1_);
		ar & BOOST_SERIALIZATION_NVP(k2_);
		ar & BOOST_SERIALIZATION_NVP(k3_);
		ar & BOOST_SERIALIZATION_NVP(k4_);
	}


	/// @}

};

}

