/**
 *  @file  BearingFactor.H
 *  @authors Frank Dellaert
 **/

#pragma once

#include "Rot2.h"
#include "Pose2.h"
#include "Point2.h"
#include "NonlinearFactor.h"

namespace gtsam {

	/**
	 * Binary factor for a bearing measurement
	 */
	template<class Config, class PoseKey, class PointKey>
	class BearingFactor: public NonlinearFactor2<Config, PoseKey, Pose2,
	PointKey, Point2> {
	private:

		Rot2 z_; /** measurement */

		typedef NonlinearFactor2<Config, PoseKey, Pose2, PointKey, Point2> Base;

	public:

		BearingFactor(); /* Default constructor */
		BearingFactor(const PoseKey& i, const PointKey& j, const Rot2& z,
				const SharedGaussian& model) :
					Base(model, i, j), z_(z) {
		}

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Pose2& pose, const Point2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			Rot2 hx = bearing(pose, point, H1, H2);
			return logmap(between(z_, hx));
		}

		/** return the measured */
		inline const Rot2 measured() const {
			return z_;
		}
	}; // BearingFactor

} // namespace gtsam
