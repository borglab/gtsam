/**
 *  @file  BearingFactor.H
 *  @authors Frank Dellaert
 **/

#pragma once

#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

	/**
	 * Binary factor for a bearing measurement
	 */
	template<class Config, class PoseKey, class PointKey>
	class BearingFactor: public NonlinearFactor2<Config, PoseKey, PointKey> {
	private:

		Rot2 z_; /** measurement */

		typedef NonlinearFactor2<Config, PoseKey, PointKey> Base;

	public:

		BearingFactor(); /* Default constructor */
		BearingFactor(const PoseKey& i, const PointKey& j, const Rot2& z,
				const SharedGaussian& model) :
					Base(model, i, j), z_(z) {
		}

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Pose2& pose, const Point2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			Rot2 hx = pose.bearing(point, H1, H2);
			return logmap(between(z_, hx));
		}

		/** return the measured */
		inline const Rot2 measured() const {
			return z_;
		}
	}; // BearingFactor

} // namespace gtsam
