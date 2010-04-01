/*
 * BearingRangeFactor.h
 *
 *   Created on: Apr 1, 2010
 *       Author: Kai Ni
 *  Description: a single factor contains both the bearing and the range to prevent handle to pair bearing and range factors
 */

#pragma once

#include "BearingFactor.h"
#include "RangeFactor.h"

namespace gtsam {

	/**
	 * Binary factor for a bearing measurement
	 */
	template<class Config, class PoseKey, class PointKey>
	class BearingRangeFactor: public NonlinearFactor2<Config, PoseKey, Pose2,
	PointKey, Point2> {
	private:

		// the bearing factor
		BearingFactor<Config, PoseKey, PointKey> bearing_;

		// the range factor
		RangeFactor<Config, PoseKey, PointKey> range_;

		typedef NonlinearFactor2<Config, PoseKey, Pose2, PointKey, Point2> Base;

	public:

		BearingRangeFactor(); /* Default constructor */
		BearingRangeFactor(const PoseKey& i, const PointKey& j, const std::pair<Rot2, double>& z,
				const SharedGaussian& model) :
					Base(model, i, j), bearing_(i, j, z.first, model), range_(i, j, z.second, model) {
		}

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Pose2& pose, const Point2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			boost::optional<Matrix&> H11 = H1, H21 = H1;
			boost::optional<Matrix&> H12 = H2, H22 = H2;
			Vector e1 = bearing_.evaluateError(pose, point, H11, H12);
			Vector e2 =   range_.evaluateError(pose, point, H21, H21);
			if (H1) *H1 = stack(2, &(*H11), &(*H21));
			if (H2) *H2 = stack(2, &(*H12), &(*H22));
			return concatVectors(2, &e1, &e2);
		}

		/** return the measured */
		inline const std::pair<Rot2, double> measured() const {
			return concatVectors(2, bearing_.measured(), range_.measured());
		}
	}; // BearingRangeFactor

} // namespace gtsam
