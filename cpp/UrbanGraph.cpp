/**
 * @file   UrbanGraph.cpp
 * @brief  A factor graph for the Urban problem
 * @author Frank Dellaert
 * @author Viorela Ila
 */

#include "FactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "UrbanGraph.h"
#include "UrbanMeasurement.h"
#include "UrbanOdometry.h"

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
	void UrbanGraph::addMeasurement(const boost::shared_ptr<const Matrix> &sensorMatrix, 
					double x, double y, double sigma, int i, int j) {
		Point2 z(x,y);
		sharedFactor factor(new UrbanMeasurement(sensorMatrix, z,sigma,i,j));
		push_back(factor);
	}
	;

	/* ************************************************************************* */
	void UrbanGraph::addOdometry(double dx, double yaw, double sigmadx,
			double sigmayaw, int p) {
		Vector z = Vector_(dx,0,0,yaw,0,0);
		Matrix cov = eye(6);
		cov(1,1)=sigmadx*sigmadx;
		cov(4,4)=sigmadx*sigmadx;
		sharedFactor factor(new UrbanOdometry(symbol('x',p),symbol('x',p+1),z,cov));
		push_back(factor);
	}

	/* ************************************************************************* */
	void UrbanGraph::addOriginConstraint(int p) {
		// TODO
	}

/* ************************************************************************* */
} // namespace gtsam
