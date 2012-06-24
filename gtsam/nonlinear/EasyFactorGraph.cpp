/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   EasyFactorGraph.cpp
 *  @brief  Nonlinear Factor Graph class with methods defined for safe and easy access in MATLAB
 *  @date   June 24, 2012
 *  @author Frank Dellaert
 **/

#include <gtsam/nonlinear/EasyFactorGraph.h>
#include <gtsam/linear/SimpleSPCGSolver.h>

namespace gtsam {

	EasyFactorGraph::EasyFactorGraph() {
	}

	EasyFactorGraph::EasyFactorGraph(const NonlinearFactorGraph& graph) :
			NonlinearFactorGraph(graph) {
	}

	LevenbergMarquardtOptimizer EasyFactorGraph::optimizer(
			const Values& initialEstimate, const LevenbergMarquardtParams& p) const {
		return LevenbergMarquardtOptimizer((*this), initialEstimate, p);
	}

	const Values& EasyFactorGraph::optimize(const Values& initialEstimate,
			size_t verbosity) const {
		LevenbergMarquardtParams p;
		p.verbosity = (NonlinearOptimizerParams::Verbosity) verbosity;
		return optimizer(initialEstimate, p).optimizeSafely();
	}

	const Values& EasyFactorGraph::optimizeSPCG(const Values& initialEstimate,
			size_t verbosity) const {
		LevenbergMarquardtParams p;
		p.verbosity = (NonlinearOptimizerParams::Verbosity) verbosity;
		p.linearSolverType = SuccessiveLinearizationParams::CG;
		p.iterativeParams = boost::make_shared<SimpleSPCGSolverParameters>();
		return optimizer(initialEstimate, p).optimizeSafely();
	}

	Marginals EasyFactorGraph::marginals(const Values& solution) const {
		return Marginals(*this, solution);
	}

} // namespace gtsam

