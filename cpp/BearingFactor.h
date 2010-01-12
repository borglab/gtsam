/**
 *  @file  BearingFactor.H
 *  @authors Frank Dellaert
 **/

#pragma once

#include "Rot2.h"
#include "Pose2.h"
#include "Point2.h"

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
		double x = d.x(), y = d.y();
		double d2 = x * x + y * y;
		double n = sqrt(d2);
		if (H) *H = Matrix_(1, 2, -y / d2, x / d2);
		return Rot2(x / n, y / n);
	}

	/** old style derivative */
	inline Matrix DrelativeBearing(const Point2& d) {
		Matrix H; relativeBearing(d, H); return H;
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
		if (!H1 && !H2) return bearing(pose,point);
		Point2 d = transform_to(pose, point);
		Matrix D_result_d;
		Rot2 result = relativeBearing(d, D_result_d);
		if (H1) *H1 = D_result_d * Dtransform_to1(pose, point);
		if (H2) *H2 = D_result_d * Dtransform_to2(pose, point);
		return result;
	}

	/** old style derivative */
	inline Matrix Dbearing1(const Pose2& pose, const Point2& point) {
		Matrix H; bearing(pose, point, H, boost::none); return H;
	}

	/** old style derivative */
	inline Matrix Dbearing2(const Pose2& pose, const Point2& point) {
		Matrix H; bearing(pose, point, boost::none, H); return H;
	}

} // namespace gtsam
