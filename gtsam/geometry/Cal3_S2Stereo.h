/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2Stereo.h
 * @brief  The most common 5DOF 3D->2D calibration + Stereo baseline
 * @author Chris Beall
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam {

	/**
	 * @brief The most common 5DOF 3D->2D calibration, stereo version
	 * @addtogroup geometry
	 * \nosubgrouping
	 */
	class Cal3_S2Stereo {
	private:

		Cal3_S2 K_;
		double b_;

	public:

		typedef boost::shared_ptr<Cal3_S2Stereo> shared_ptr;  ///< shared pointer to stereo calibration object

    /// @name Standard Constructors
    /// @

		/// default calibration leaves coordinates unchanged
		Cal3_S2Stereo() :
			K_(1, 1, 0, 0, 0), b_(1.0) {
		}

		/// constructor from doubles
		Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b) :
			K_(fx, fy, s, u0, v0), b_(b) {
		}

		/// @}
		/// @name Testable
		/// @{

		void print(const std::string& s = "") const {
			K_.print(s+"K: ");
			std::cout << s << "Baseline: " << b_ << std::endl;
		}

		/// Check if equal up to specified tolerance
		bool equals(const Cal3_S2Stereo& other, double tol = 10e-9) const {
			if (fabs(b_ - other.b_) > tol) return false;
			return K_.equals(other.K_,tol);
		}

   /// @}
    /// @name Standard Interface
    /// @{

		/// return calibration, same for left and right
		const Cal3_S2& calibration() const { return K_;}

		/// return calibration matrix K, same for left and right
		Matrix matrix() const { return K_.matrix();}

		/// focal length x
		inline double fx() const { return K_.fx();}

		/// focal length x
		inline double fy() const { return K_.fy();}

		/// skew
		inline double skew() const { return K_.skew();}

		/// image center in x
		inline double px() const { return K_.px();}

		/// image center in y
		inline double py() const { return K_.py();}

		/// return the principal point
		Point2 principalPoint() const { return K_.principalPoint();}

		/// return baseline
		inline double baseline() const { return b_; }

    /// @}
    /// @name Advanced Interface
		/// @{

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & BOOST_SERIALIZATION_NVP(K_);
			ar & BOOST_SERIALIZATION_NVP(b_);
		}
		/// @}

	};

	typedef boost::shared_ptr<Cal3_S2Stereo> shared_ptrKStereo;  ///< shared pointer to stereo calibration object

}
