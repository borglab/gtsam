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

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam {

	/** The most common 5DOF 3D->2D calibration */
	class Cal3_S2Stereo: public Cal3_S2 {
	private:
		double b_;

	public:
		/**
		 * default calibration leaves coordinates unchanged
		 */
		Cal3_S2Stereo() :
			Cal3_S2(1, 1, 0, 0, 0), b_(1.0) {
		}

		/**
		 * constructor from doubles
		 */
		Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b) :
			Cal3_S2(fx, fy, s, u0, v0), b_(b) {
		}

		void print(const std::string& s = "") const {
			gtsam::print(matrix(), s);
			std::cout << "Baseline: " << b_ << std::endl;
		}

//		/**
//		 * Check if equal up to specified tolerance
//		 */
//		bool equals(const Cal3_S2& K, double tol = 10e-9) const;

		inline double baseline() const { return b_; }


	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & BOOST_SERIALIZATION_NVP(b_);
		}
	};

	typedef boost::shared_ptr<Cal3_S2Stereo> shared_ptrKStereo;

}
