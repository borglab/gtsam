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

#include <gtsam/geometry/Point2.h>

namespace gtsam {

	/**
	 * @brief Calibration of a camera with radial distortion
	 * @ingroup geometry
	 */
	class Cal3DS2 : Testable<Cal3DS2> {
	private:
		double fx_, fy_, s_, u0_, v0_ ; // focal length, skew and principal point
		double k1_, k2_ ; // radial 2nd-order and 4th-order
		double k3_, k4_ ; // tagential distortion

		// K = [ fx s u0 ; 0 fy v0 ; 0 0 1 ]
		// r = Pn.x^2 + Pn.y^2
		// \hat{pn} = (1 + k1*r + k2*r^2 ) pn + [ 2*k3 pn.x pn.y + k4 (r + 2 Pn.x^2) ;
		//										  k3 (r + 2 Pn.y^2) + 2*k4 pn.x pn.y  ]
		// pi = K*pn

	public:
		// Default Constructor with only unit focal length
		Cal3DS2();

		// Construction
		Cal3DS2(double fx, double fy, double s, double u0, double v0,
				double k1, double k2, double k3, double k4) ;

		Cal3DS2(const Vector &v) ;

		Matrix K() const ;
		Vector k() const ;
		Vector vector() const ;
		void print(const std::string& s = "") const ;
		bool equals(const Cal3DS2& K, double tol = 10e-9) const;


		Point2 uncalibrate(const Point2& p,
						   boost::optional<Matrix&> H1 = boost::none,
						   boost::optional<Matrix&> H2 = boost::none) const ;

		Matrix D2d_intrinsic(const Point2& p) const ;
		Matrix D2d_calibration(const Point2& p) const ;

		Cal3DS2 expmap(const Vector& d) const ;
		Vector logmap(const Cal3DS2& T2) const ;

	    static Cal3DS2 Expmap(const Vector& v) ;
		static Vector Logmap(const Cal3DS2& p) ;

		int dim() const { return 9 ; }
		static size_t Dim() { return 9; }

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
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

	};
}

