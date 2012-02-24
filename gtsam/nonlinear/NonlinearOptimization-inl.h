/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearOptimization-inl.h
 * @date Oct 17, 2010
 * @author Kai Ni
 * @brief Easy interfaces for NonlinearOptimizer
 */

#pragma once

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

#if ENABLE_SPCG
#include <gtsam/linear/SubgraphSolver.h>
#endif

#include <gtsam/nonlinear/NonlinearOptimizer.h>

using namespace std;

namespace gtsam {

	/**
	 * The Elimination solver
	 */
	template<class G>
	Values	optimizeSequential(const G& graph, const Values& initialEstimate,
			const NonlinearOptimizationParameters& parameters, bool useLM) {

		// Use a variable ordering from COLAMD
	  Ordering::shared_ptr ordering = graph.orderingCOLAMD(initialEstimate);

		// Create an nonlinear Optimizer that uses a Sequential Solver
	  typedef NonlinearOptimizer<G, GaussianFactorGraph, GaussianSequentialSolver> Optimizer;
	  Optimizer optimizer(boost::make_shared<const G>(graph),
	  		boost::make_shared<const Values>(initialEstimate), ordering,
	  		boost::make_shared<NonlinearOptimizationParameters>(parameters));

	  // Now optimize using either LM or GN methods.
		if (useLM)
			return *optimizer.levenbergMarquardt().values();
		else
			return *optimizer.gaussNewton().values();
	}

	/**
	 * The multifrontal solver
	 */
	template<class G>
	Values	optimizeMultiFrontal(const G& graph, const Values& initialEstimate,
			const NonlinearOptimizationParameters& parameters, bool useLM) {

		// Use a variable ordering from COLAMD
	  Ordering::shared_ptr ordering = graph.orderingCOLAMD(initialEstimate);

		// Create an nonlinear Optimizer that uses a Multifrontal Solver
	  typedef NonlinearOptimizer<G, GaussianFactorGraph, GaussianMultifrontalSolver> Optimizer;
	  Optimizer optimizer(boost::make_shared<const G>(graph),
	  		boost::make_shared<const Values>(initialEstimate), ordering,
	  		boost::make_shared<NonlinearOptimizationParameters>(parameters));

	  // now optimize using either LM or GN methods
		if (useLM)
			return *optimizer.levenbergMarquardt().values();
		else
			return *optimizer.gaussNewton().values();
	}

#if ENABLE_SPCG
	/**
	 * The sparse preconditioned conjugate gradient solver
	 */
	template<class G>
	Values	optimizeSPCG(const G& graph, const Values& initialEstimate,
			const NonlinearOptimizationParameters& parameters = NonlinearOptimizationParameters(),
			bool useLM = true) {

		// initial optimization state is the same in both cases tested
		typedef SubgraphSolver<G,GaussianFactorGraph,Values> Solver;
		typedef boost::shared_ptr<Solver> shared_Solver;
		typedef NonlinearOptimizer<G, GaussianFactorGraph, Solver> SPCGOptimizer;
		shared_Solver solver = boost::make_shared<Solver>(
				graph, initialEstimate, IterativeOptimizationParameters());
		SPCGOptimizer optimizer(
				boost::make_shared<const G>(graph),
				boost::make_shared<const Values>(initialEstimate),
				solver->ordering(),
				solver,
				boost::make_shared<NonlinearOptimizationParameters>(parameters));

	  // choose nonlinear optimization method
		if (useLM)
			return *optimizer.levenbergMarquardt().values();
		else
			return *optimizer.gaussNewton().values();
	}
#endif

	/**
	 * optimization that returns the values
	 */
	template<class G>
	Values optimize(const G& graph, const Values& initialEstimate, const NonlinearOptimizationParameters& parameters,
			const LinearSolver& solver,
			const NonlinearOptimizationMethod& nonlinear_method) {
		switch (solver) {
		case SEQUENTIAL:
			return optimizeSequential<G>(graph, initialEstimate, parameters,
					nonlinear_method == LM);
		case MULTIFRONTAL:
			return optimizeMultiFrontal<G>(graph, initialEstimate, parameters,
					nonlinear_method == LM);
#if ENABLE_SPCG
		case SPCG:
//			return optimizeSPCG<G,Values>(graph, initialEstimate, parameters,
//								nonlinear_method == LM);
			throw runtime_error("optimize: SPCG not supported yet due to the specific pose constraint");
#endif
		}
		throw runtime_error("optimize: undefined solver");
	}

} //namespace gtsam
