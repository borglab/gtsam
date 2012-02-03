/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BearingFactor.H
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

	/**
	 * Binary factor for a bearing measurement
	 */
	template<class POSEKEY, class POINTKEY>
	class BearingFactor: public NonlinearFactor2<POSEKEY, POINTKEY> {
	private:

		typedef typename POSEKEY::Value Pose;
		typedef typename POSEKEY::Value::Rotation Rot;
		typedef typename POINTKEY::Value Point;

		typedef BearingFactor<POSEKEY, POINTKEY> This;
		typedef NonlinearFactor2<POSEKEY, POINTKEY> Base;

		Rot z_; /** measurement */

		/** concept check by type */
		GTSAM_CONCEPT_TESTABLE_TYPE(Rot)
		GTSAM_CONCEPT_POSE_TYPE(Pose)

	public:

		/** default constructor for serialization/testing only */
		BearingFactor() {}

		/** primary constructor */
		BearingFactor(const POSEKEY& i, const POINTKEY& j, const Rot& z,
				const SharedNoiseModel& model) :
					Base(model, i, j), z_(z) {
		}

		virtual ~BearingFactor() {}

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Pose& pose, const Point& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			Rot hx = pose.bearing(point, H1, H2);
			return Rot::Logmap(z_.between(hx));
		}

		/** return the measured */
		inline const Rot measured() const {
			return z_;
		}

		/** equals */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e =	dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) && this->z_.equals(e->z_, tol);
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

	}; // BearingFactor

} // namespace gtsam
