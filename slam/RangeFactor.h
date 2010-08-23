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
	template<class Config, class PoseKey, class PointKey>
	class RangeFactor: public NonlinearFactor2<Config, PoseKey, PointKey> {
	private:

		double z_; /** measurement */

		typedef NonlinearFactor2<Config, PoseKey, PointKey> Base;

	public:

		RangeFactor(); /* Default constructor */

		RangeFactor(const PoseKey& i, const PointKey& j, double z,
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
	}; // RangeFactor

} // namespace gtsam
