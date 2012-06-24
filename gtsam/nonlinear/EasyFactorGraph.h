/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   EasyFactorGraph.h
 *  @brief  Nonlinear Factor Graph class with methods defined for safe and easy access in MATLAB
 *  @date   June 24, 2012
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

namespace gtsam {

	/**
	 * Nonlinear Factor Graph class with methods defined for safe and easy access in MATLAB
	 */
	struct EasyFactorGraph: public NonlinearFactorGraph {

		/// Default constructor for a NonlinearFactorGraph
		EasyFactorGraph();

		/// Creates a NonlinearFactorGraph based on another NonlinearFactorGraph
		EasyFactorGraph(const NonlinearFactorGraph& graph);

		/**
		 *  Setup and return a LevenbargMarquardtOptimizer
		 *  @param initialEstimate initial estimate of all variables in the graph
		 *  @param p optimizer's parameters
		 *  @return a LevenbergMarquardtOptimizer object, which you can use to control the optimization process
		 */
		LevenbergMarquardtOptimizer optimizer(const Values& initialEstimate,
				const LevenbergMarquardtParams& p = LevenbergMarquardtParams()) const;

		/**
		 *  Safely optimize the graph, using Levenberg-Marquardt
		 *  @param initialEstimate initial estimate of all variables in the graph
		 *  @param verbosity unsigned specifying verbosity level
		 */
		const Values& optimize(const Values& initialEstimate, size_t verbosity = 0) const;

		/**
		 *  Safely optimize using subgraph preconditioning
		 *  @param initialEstimate initial estimate of all variables in the graph
		 *  @param verbosity unsigned specifying verbosity level
		 */
		const Values& optimizeSPCG(const Values& initialEstimate, size_t verbosity = 0) const;

		/**
		 *  Return a Marginals object, so you can query marginals
		 */
		Marginals marginals(const Values& solution) const;

	};

} // namespace gtsam

