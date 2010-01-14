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
	 * Calculate relative bearing to a landmark in local coordinate frame
	 * @param point 2D location of landmark
	 * @param H optional reference for Jacobian
	 * @return 2D rotation \in SO(2)
	 */
	Rot2 relativeBearing(const Point2& d) {
		double n = d.norm();
		return Rot2(d.x() / n, d.y() / n);
	}

	/**
	 * Calculate relative bearing and optional derivative
	 */
	Rot2 relativeBearing(const Point2& d, boost::optional<Matrix&> H) {
		double x = d.x(), y = d.y(), d2 = x * x + y * y, n = sqrt(d2);
		if (H) *H = Matrix_(1, 2, -y / d2, x / d2);
		return Rot2(x / n, y / n);
	}

	/**
	 * Calculate bearing to a landmark
	 * @param pose 2D pose of robot
	 * @param point 2D location of landmark
	 * @return 2D rotation \in SO(2)
	 */
	Rot2 bearing(const Pose2& pose, const Point2& point) {
		Point2 d = transform_to(pose, point);
		return relativeBearing(d);
	}

	/**
	 * Calculate bearing and optional derivative(s)
	 */
	Rot2 bearing(const Pose2& pose, const Point2& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
		if (!H1 && !H2) return bearing(pose, point);
		Point2 d = transform_to(pose, point);
		Matrix D_result_d;
		Rot2 result = relativeBearing(d, D_result_d);
		if (H1) *H1 = D_result_d * Dtransform_to1(pose, point);
		if (H2) *H2 = D_result_d * Dtransform_to2(pose, point);
		return result;
	}

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement,
	 * i.e. the main building block for visual SLAM.
	 */
	template<class Config, class PoseKey, class PointKey>
	class BearingFactor: public NonlinearFactor2<Config, PoseKey, Pose2,
			PointKey, Point2> {
	private:

		Rot2 z_; /** measurement */

		typedef NonlinearFactor2<Config, PoseKey, Pose2, PointKey, Point2> Base;

	public:

		BearingFactor(); /* Default constructor */
		BearingFactor(const Rot2& z, double sigma, const PoseKey& i,
				const PointKey& j) :
			Base(sigma, i, j), z_(z) {
		}

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Pose2& pose, const Point2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			Rot2 hx = bearing(pose, point, H1, H2);
			return logmap(between(z_, hx));
		}
	}; // BearingFactor

} // namespace gtsam
