/**
 * @file    simulated2D.cpp
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

#include "simulated2D.h"

namespace simulated2D {

	static Matrix I = gtsam::eye(2);

	/* ************************************************************************* */
	Vector prior(const Vector& x, boost::optional<Matrix&> H) {
		if (H) *H = I;
		return x;
	}

	/* ************************************************************************* */
	Vector odo(const Vector& x1, const Vector& x2, boost::optional<Matrix&> H1,
			boost::optional<Matrix&> H2) {
		if (H1) *H1 = -I;
		if (H2) *H2 = I;
		return x2 - x1;
	}

	/* ************************************************************************* */
	Vector mea(const Vector& x, const Vector& l, boost::optional<Matrix&> H1,
			boost::optional<Matrix&> H2) {
		if (H1) *H1 = -I;
		if (H2) *H2 = I;
		return l - x;
	}

/* ************************************************************************* */

} // namespace simulated2D
