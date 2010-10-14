/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
	template<class Values, class PoseKey, class PointKey>
	class BearingFactor: public NonlinearFactor2<Values, PoseKey, PointKey> {
	private:

		Rot2 z_; /** measurement */

		typedef NonlinearFactor2<Values, PoseKey, PointKey> Base;

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
			return Rot2::Logmap(z_.between(hx));
		}

		/** return the measured */
		inline const Rot2 measured() const {
			return z_;
		}
	}; // BearingFactor

} // namespace gtsam
