/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BearingRangeFactor.h
 * @date Apr 1, 2010
 * @author Kai Ni
 * @brief a single factor contains both the bearing and the range to prevent handle to pair bearing and range factors
 */

#pragma once

#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

	/**
	 * Binary factor for a bearing measurement
	 */
	template<class VALUES, class POSEKEY, class POINTKEY>
	class BearingRangeFactor: public NonlinearFactor2<VALUES, POSEKEY, POINTKEY> {
	private:

		typedef typename POSEKEY::Value Pose;
		typedef typename POSEKEY::Value::Rotation Rot;
		typedef typename POINTKEY::Value Point;

		typedef BearingRangeFactor<VALUES, POSEKEY, POINTKEY> This;
		typedef NonlinearFactor2<VALUES, POSEKEY, POINTKEY> Base;

		// the measurement
		Rot bearing_;
		double range_;

		/** concept check by type */
		GTSAM_CONCEPT_TESTABLE_TYPE(Rot)
		GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(Pose, Point)
		GTSAM_CONCEPT_POSE_TYPE(Pose)

	public:

		BearingRangeFactor() {} /* Default constructor */
		BearingRangeFactor(const POSEKEY& i, const POINTKEY& j, const Rot& bearing, const double range,
				const SharedNoiseModel& model) :
					Base(model, i, j), bearing_(bearing), range_(range) {
		}

		virtual ~BearingRangeFactor() {}

		/** h(x)-z -> between(z,h(x)) for Rot manifold */
		Vector evaluateError(const Pose& pose, const Point& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			Matrix H11, H21, H12, H22;
			boost::optional<Matrix&> H11_ = H1 ? boost::optional<Matrix&>(H11) : boost::optional<Matrix&>();
			boost::optional<Matrix&> H21_ = H1 ? boost::optional<Matrix&>(H21) : boost::optional<Matrix&>();
			boost::optional<Matrix&> H12_ = H2 ? boost::optional<Matrix&>(H12) : boost::optional<Matrix&>();
			boost::optional<Matrix&> H22_ = H2 ? boost::optional<Matrix&>(H22) : boost::optional<Matrix&>();

			Rot y1 = pose.bearing(point, H11_, H12_);
			Vector e1 = Rot::Logmap(bearing_.between(y1));

			double y2 = pose.range(point, H21_, H22_);
			Vector e2 = Vector_(1, y2 - range_);

			if (H1) *H1 = gtsam::stack(2, &H11, &H21);
			if (H2) *H2 = gtsam::stack(2, &H12, &H22);
			return concatVectors(2, &e1, &e2);
		}

		/** return the measured */
		inline const std::pair<Rot, double> measured() const {
			return std::make_pair(bearing_, range_);
		}

		/** equals */
		virtual bool equals(const NonlinearFactor<VALUES>& expected, double tol=1e-9) const {
			const This *e =	dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) &&
					fabs(this->range_ - e->range_) < tol &&
					this->bearing_.equals(e->bearing_, tol);
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor2",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(bearing_);
			ar & BOOST_SERIALIZATION_NVP(range_);
		}
	}; // BearingRangeFactor

} // namespace gtsam
