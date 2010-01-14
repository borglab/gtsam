/**
 *  @file  RangeFactor.H
 *  @authors Frank Dellaert
 **/

#pragma once

#include "Pose2.h"
#include "Point2.h"
#include "NonlinearFactor.h"

namespace gtsam {

	/**
	 * Calculate range to a landmark
	 * @param pose 2D pose of robot
	 * @param point 2D location of landmark
	 * @return range (double)
	 */
	double range(const Pose2& pose, const Point2& point) {
		Point2 d = transform_to(pose, point);
		return d.norm();
	}

	/**
	 * Calculate range and optional derivative(s)
	 */
	double range(const Pose2& pose, const Point2& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
		if (!H1 && !H2) return range(pose, point);
		Point2 d = transform_to(pose, point);
		double x = d.x(), y = d.y(), d2 = x * x + y * y, n = sqrt(d2);
		Matrix D_result_d = Matrix_(1, 2, x / n, y / n);
		if (H1) *H1 = D_result_d * Dtransform_to1(pose, point);
		if (H2) *H2 = D_result_d * Dtransform_to2(pose, point);
		return n;
	}

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement,
	 * i.e. the main building block for visual SLAM.
	 */
	template<class Config, class PoseKey, class PointKey>
	class RangeFactor: public NonlinearFactor2<Config, PoseKey, Pose2, PointKey,
			Point2> {
	private:

		double z_; /** measurement */

		typedef NonlinearFactor2<Config, PoseKey, Pose2, PointKey, Point2> Base;

	public:

		RangeFactor(); /* Default constructor */

		RangeFactor(double z, double sigma, const PoseKey& i, const PointKey& j) :
			Base(sigma, i, j), z_(z) {
		}

		/** h(x)-z */
		Vector evaluateError(const Pose2& pose, const Point2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			double hx = gtsam::range(pose, point, H1, H2);
			return Vector_(1, hx - z_);
		}
	}; // RangeFactor

} // namespace gtsam
