/**
 * @file    simulated2D.cpp
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

#include "simulated2D.h"
#include "LieConfig-inl.h"
#include "TupleConfig-inl.h"

namespace gtsam {

	using namespace simulated2D;
	INSTANTIATE_LIE_CONFIG(PointKey, Point2)
	INSTANTIATE_LIE_CONFIG(PoseKey, Point2)
	INSTANTIATE_TUPLE_CONFIG2(PoseConfig, PointConfig)
//	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(Config)
//	INSTANTIATE_NONLINEAR_OPTIMIZER(Graph, Config)

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
