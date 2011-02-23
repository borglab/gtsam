/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  RangeFactor.H
 *  @authors Frank Dellaert
 **/

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

	/**
	 * Binary factor for a range measurement
	 */
	template<class VALUES, class POSEKEY, class POINTKEY>
	class RangeFactor: public NonlinearFactor2<VALUES, POSEKEY, POINTKEY> {
	private:

		double z_; /** measurement */

		typedef RangeFactor<VALUES, POSEKEY, POINTKEY> This;
		typedef NonlinearFactor2<VALUES, POSEKEY, POINTKEY> Base;

	public:

		RangeFactor() {} /* Default constructor */

		RangeFactor(const POSEKEY& i, const POINTKEY& j, double z,
				const SharedGaussian& model) :
					Base(model, i, j), z_(z) {
		}

		/** h(x)-z */
		Vector evaluateError(const Pose2& pose, const Point2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			double hx = pose.range(point, H1, H2);
			return Vector_(1, hx - z_);
		}

		/** return the measured */
		inline double measured() const {
			return z_;
		}

		/** equals specialized to this factor */
		virtual bool equals(const NonlinearFactor<VALUES>& expected, double tol=1e-9) const {
			const This *e = dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) && fabs(this->z_ - e->z_) < tol;
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor2",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(z_);
		}
	}; // RangeFactor

} // namespace gtsam
