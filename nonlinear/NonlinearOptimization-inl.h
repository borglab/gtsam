/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * NonlinearOptimization-inl.h
 *
 *   Created on: Oct 17, 2010
 *       Author: Kai Ni
 *  Description: Easy interfaces for NonlinearOptimizer
 */

#pragma once

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/linear/SubgraphSolver-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>

using namespace std;

namespace gtsam {

	/**
	 * The Elimination solver
	 */
	template<class G, class T>
	T	optimizeSequential(const G& graph, const T& initialEstimate, const NonlinearOptimizationParameters& parameters) {

		// Use a variable ordering from COLAMD
	  Ordering::shared_ptr ordering = graph.orderingCOLAMD(initialEstimate);

		// initial optimization state is the same in both cases tested
	  typedef NonlinearOptimizer<G, T, GaussianFactorGraph, GaussianSequentialSolver> Optimizer;
	  Optimizer optimizer(boost::make_shared<const G>(graph),
	  		boost::make_shared<const T>(initialEstimate), ordering);

		// Levenberg-Marquardt
	  Optimizer result = optimizer.levenbergMarquardt(parameters);
		return *result.values();
	}

	/**
	 * The multifrontal solver
	 */
	template<class G, class T>
	T	optimizeMultiFrontal(const G& graph, const T& initialEstimate, const NonlinearOptimizationParameters& parameters) {

		// Use a variable ordering from COLAMD
	  Ordering::shared_ptr ordering = graph.orderingCOLAMD(initialEstimate);

		// initial optimization state is the same in both cases tested
	  typedef NonlinearOptimizer<G, T, GaussianFactorGraph, GaussianMultifrontalSolver> Optimizer;
	  Optimizer optimizer(boost::make_shared<const G>(graph),
	  		boost::make_shared<const T>(initialEstimate), ordering);

		// Levenberg-Marquardt
	  Optimizer result = optimizer.levenbergMarquardt(parameters);
		return *result.values();
	}

	/**
	 * The sparse preconditioned conjucate gradient solver
	 */
	template<class G, class T>
	T	optimizeSPCG(const G& graph, const T& initialEstimate, const NonlinearOptimizationParameters& parameters = NonlinearOptimizationParameters()) {

		// initial optimization state is the same in both cases tested
		typedef SubgraphSolver<G,GaussianFactorGraph,T> Solver;
		typedef boost::shared_ptr<Solver> shared_Solver;
		typedef NonlinearOptimizer<G, T, GaussianFactorGraph, Solver> SPCGOptimizer;
		shared_Solver solver = boost::make_shared<Solver>(graph, initialEstimate);
		SPCGOptimizer optimizer(
				boost::make_shared<const G>(graph),
				boost::make_shared<const T>(initialEstimate),
				solver->ordering(),
				solver);

		// Levenberg-Marquardt
		SPCGOptimizer result = optimizer.levenbergMarquardt(parameters);
		return *result.values();
	}

	/**
	 * optimization that returns the values
	 */
	template<class G, class T>
	T optimize(const G& graph, const T& initialEstimate, const NonlinearOptimizationParameters& parameters,
			const enum LinearSolver& solver) {
		switch (solver) {
		case SEQUENTIAL:
			return optimizeSequential<G,T>(graph, initialEstimate, parameters);
		case MULTIFRONTAL:
			return optimizeMultiFrontal<G,T>(graph, initialEstimate, parameters);
		case SPCG:
			throw runtime_error("optimize: SPCG not supported yet due to the specific pose constraint");

			break;
		}
		throw runtime_error("optimize: undefined solver");
	}

} //namespace gtsam
