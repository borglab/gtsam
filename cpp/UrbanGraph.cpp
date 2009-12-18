/**
 * @file   UrbanGraph.cpp
 * @brief  A factor graph for the Urban problem
 * @author Frank Dellaert
 * @author Viorela Ila
 */

#include "FactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "UrbanGraph.h"

namespace gtsam {

	// explicit instantiation so all the code is there and we can link with it
	template class FactorGraph<UrbanFactor>;
	template class NonlinearFactorGraph<UrbanConfig>;
	template class NonlinearOptimizer<UrbanGraph,UrbanConfig>;

	/* ************************************************************************* */
	UrbanGraph::UrbanGraph() {
	}

	/* ************************************************************************* */
	void UrbanGraph::print(const std::string& s) const {
		gtsam::NonlinearFactorGraph<UrbanConfig>::print(s);
		// TODO
	}

	/* ************************************************************************* */
	bool UrbanGraph::equals(const UrbanGraph& p, double tol) const {
		return gtsam::NonlinearFactorGraph<UrbanConfig>::equals(p, tol);
		// TODO
	}

	/* ************************************************************************* */
	void UrbanGraph::addMeasurement(double x, double y, double sigma, int i, int j) {
		Point2 z(x,y);
		sharedFactor factor(new UrbanMeasurement(z,sigma,i,j));
		push_back(factor);
	}
	;

	/* ************************************************************************* */
	void UrbanGraph::addOdometry(double dx, double yaw, double sigmadx,
			double sigmayaw, int p) {
		// TODO
	}

	/* ************************************************************************* */
	void UrbanGraph::addOriginConstraint(int p) {
		// TODO
	}

/* ************************************************************************* */
} // namespace gtsam
