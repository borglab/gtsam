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
	 * @addtogroup SLAM
	 */
	template<class POSE, class POINT, class ROTATION = typename POSE::Rotation>
	class BearingFactor: public NoiseModelFactor2<POSE, POINT> {
	private:

		typedef POSE Pose;
		typedef ROTATION Rot;
		typedef POINT Point;

		typedef BearingFactor<POSE, POINT> This;
		typedef NoiseModelFactor2<POSE, POINT> Base;

		Rot measured_; /** measurement */

		/** concept check by type */
		GTSAM_CONCEPT_TESTABLE_TYPE(Rot)
		GTSAM_CONCEPT_POSE_TYPE(Pose)

	public:

		/** default constructor for serialization/testing only */
		BearingFactor() {}

		/** primary constructor */
		BearingFactor(Key poseKey, Key pointKey, const Rot& measured,
				const SharedNoiseModel& model) :
					Base(model, poseKey, pointKey), measured_(measured) {
		}

		virtual ~BearingFactor() {}

		/// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
		      gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Pose& pose, const Point& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			Rot hx = pose.bearing(point, H1, H2);
			return Rot::Logmap(measured_.between(hx));
		}

		/** return the measured */
		const Rot& measured() const {
			return measured_;
		}

		/** equals */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e =	dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) && this->measured_.equals(e->measured_, tol);
		}

    /** print contents */
    void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BearingFactor, bearing = ";
      measured_.print();
      Base::print("", keyFormatter);
    }

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NoiseModelFactor2",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(measured_);
		}

	}; // BearingFactor

} // namespace gtsam
