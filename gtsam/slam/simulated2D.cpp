/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    simulated2D.cpp
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

#include <gtsam/slam/simulated2D.h>
#include <gtsam/nonlinear/LieValues-inl.h>
#include <gtsam/nonlinear/TupleValues-inl.h>

namespace gtsam {

	using namespace simulated2D;

	INSTANTIATE_LIE_CONFIG(PoseKey)
	INSTANTIATE_TUPLE_CONFIG2(PoseValues, PointValues)

	namespace simulated2D {

		static Matrix I = gtsam::eye(2);

		/* ************************************************************************* */
		Point2 prior(const Point2& x, boost::optional<Matrix&> H) {
			if (H) *H = I;
			return x;
		}

		/* ************************************************************************* */
		Point2 odo(const Point2& x1, const Point2& x2, boost::optional<Matrix&> H1,
				boost::optional<Matrix&> H2) {
			if (H1) *H1 = -I;
			if (H2) *H2 = I;
			return x2 - x1;
		}

		/* ************************************************************************* */
		Point2 mea(const Point2& x, const Point2& l, boost::optional<Matrix&> H1,
				boost::optional<Matrix&> H2) {
			if (H1) *H1 = -I;
			if (H2) *H2 = I;
			return l - x;
		}

	/* ************************************************************************* */

	} // namespace simulated2D
} // namespace gtsam
