/**
 * @file    NonlinearFactorGraph.h
 * @brief   Factor Graph Constsiting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include "LinearFactorGraph.h"
#include "FactorGraph.h"
#include "NonlinearFactor.h"
#include "ChordalBayesNet.h"
#include <colamd/colamd.h>

namespace gtsam {

/** Factor Graph Constsiting of non-linear factors */
class NonlinearFactorGraph : public FactorGraph<NonlinearFactor>
{
public: // internal, exposed for testing only, doc in .cpp file

	FGConfig iterate(const FGConfig& config, const Ordering& ordering) const;

	double iterate(FGConfig& config, const Ordering& ordering, int verbosity) const;

	std::pair<FGConfig,double> try_lambda
	(const FGConfig& config, const LinearFactorGraph& linear,
			double lambda, const Ordering& ordering, int verbosity) const;

	double iterateLM(FGConfig& config, double currentError,
			double& lambda, double lambdaFactor,
			const Ordering& ordering, int verbosity) const;

	Ordering getOrdering(FGConfig& config);

public: // these you will probably want to use
	/**
	 * linearize a non linear factor
	 */
	LinearFactorGraph linearize(const FGConfig& config) const;

	/**
	 * Optimize a solution for a non linear factor graph
	 * Changes config in place
	 * @param config       Reference to current configuration
	 * @param ordering     Ordering to optimize with
	 * @param relativeErrorTreshold
	 * @param absoluteErrorTreshold
	 * @param verbosity Integer specifying how much output to provide
	 */
	void optimize(FGConfig& config,
			const Ordering& ordering,
			double relativeErrorTreshold, double absoluteErrorTreshold,
			int verbosity = 0) const;

	/**
	 * Given two configs, check whether the error of config2 is
	 * different enough from the error for config1, or whether config1
	 * is essentially optimal
	 *
	 * @param config1  Reference to first configuration
	 * @param config2  Reference to second configuration
	 * @param relativeErrorTreshold
	 * @param absoluteErrorTreshold
	 * @param verbosity Integer specifying how much output to provide
	 */
	bool check_convergence(const FGConfig& config1,
			const FGConfig& config2,
			double relativeErrorTreshold, double absoluteErrorTreshold,
			int verbosity = 0);

	/**
	 * Optimize using Levenberg-Marquardt. Really Levenberg's
	 * algorithm at this moment, as we just add I*\lambda to Hessian
	 * H'H. The probabilistic explanation is very simple: every
	 * variable gets an extra Gaussian prior that biases staying at
	 * current value, with variance 1/lambda. This is done very easily
	 * (but perhaps wastefully) by adding a prior factor for each of
	 * the variables, after linearization.
	 *
	 * Changes config in place :-(
	 *
	 * @param config       Reference to current configuration
	 * @param ordering     Ordering to optimize with
	 * @param relativeErrorTreshold
	 * @param absoluteErrorTreshold
	 * @param verbosity    Integer specifying how much output to provide
	 * @param lambda       Reference to current lambda
	 * @param lambdaFactor Factor by which to decrease/increase lambda
	 */
	void optimizeLM(FGConfig& config,
			const Ordering& ordering,
			double relativeErrorTreshold, double absoluteErrorTreshold,
			int verbosity = 0,
			double lambda0=1e-5, double lambdaFactor=10) const;



	std::pair<LinearFactorGraph, FGConfig> OneIterationLM( FGConfig& config, const Ordering& ordering,
			double relativeErrorTreshold,
			double absoluteErrorTreshold,
			int verbosity,
			double lambda0,
			double lambdaFactor) ;

};
}
