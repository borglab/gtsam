/**
 * @file   VSLAMConfig.cpp
 * @brief  The Config
 * @author Alireza Fathi
 * @author Carlos Nieto
 */

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "VSLAMConfig.h"
#include "LieConfig-inl.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	// Exponential map
	VSLAMConfig expmap(const VSLAMConfig& x0, const VectorConfig & delta) {
		VSLAMConfig x;
		x.poses_ = expmap(x0.poses_, delta);
		x.points_ = expmap(x0.points_, delta);
		return x;
	}

	/* ************************************************************************* */
	void VSLAMConfig::print(const std::string& s) const {
		printf("%s:\n", s.c_str());
		poses_.print("Poses");
		points_.print("Points");
	}

	/* ************************************************************************* */
	bool VSLAMConfig::equals(const VSLAMConfig& c, double tol) const {
		return poses_.equals(c.poses_, tol) && points_.equals(c.points_, tol);
	}

/* ************************************************************************* */

} // namespace gtsam

