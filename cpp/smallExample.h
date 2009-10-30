/**
 * @file    smallExample.h
 * @brief   Create small example with two poses and one landmark
 * @brief   smallExample
 * @author  Carlos Nieto
 */

// \callgraph


#pragma once

#include <boost/shared_ptr.hpp>
#include "NonlinearFactorGraph.h"

namespace gtsam {

	class ConstrainedLinearFactorGraph;

	typedef NonlinearFactorGraph<VectorConfig> ExampleNonlinearFactorGraph;

	/**
	 * Create small example for non-linear factor graph
	 */
	boost::shared_ptr<const ExampleNonlinearFactorGraph > sharedNonlinearFactorGraph();
	ExampleNonlinearFactorGraph createNonlinearFactorGraph();

	/**
	 * Create configuration to go with it
	 * The ground truth configuration for the example above
	 */
	VectorConfig createConfig();

	/**
	 * create a noisy configuration for a nonlinear factor graph
	 */
	boost::shared_ptr<const VectorConfig> sharedNoisyConfig();
	VectorConfig createNoisyConfig();

	/**
	 * Zero delta config
	 */
	VectorConfig createZeroDelta();

	/**
	 * Delta config that, when added to noisyConfig, returns the ground truth
	 */
	VectorConfig createCorrectDelta();

	/**
	 * create a linear factor graph
	 * The non-linear graph above evaluated at NoisyConfig
	 */
	LinearFactorGraph createLinearFactorGraph();

	/**
	 * create small Chordal Bayes Net x <- y
	 */
	ChordalBayesNet createSmallChordalBayesNet();

	/**
	 * Create really non-linear factor graph (cos/sin)
	 */
	boost::shared_ptr<const ExampleNonlinearFactorGraph> sharedReallyNonlinearFactorGraph();
	ExampleNonlinearFactorGraph createReallyNonlinearFactorGraph();

	/**
	 * Create a Kalman smoother by linearizing a non-linear factor graph
	 * @param T number of time-steps
	 */
	LinearFactorGraph createSmoother(int T);


	/* ******************************************************* */
	// Constrained Examples
	/* ******************************************************* */

	/**
	 * Creates a simple constrained graph with one linear factor and
	 * one binary constraint.
	 */
	ConstrainedLinearFactorGraph createSingleConstraintGraph();

	/**
	 * Creates a constrained graph with a linear factor and two
	 * binary constraints that share a node
	 */
	ConstrainedLinearFactorGraph createMultiConstraintGraph();

	/**
	 * These are the old examples from the EqualityFactor/DeltaFunction
	 * They should be updated for use at some point, but are disabled for now
	 */
	/**
	 * Create configuration for constrained example
	 * This is the ground truth version
	 */
	//VectorConfig createConstrainedConfig();

	/**
	 * Create a noisy configuration for linearization
	 */
	//VectorConfig createConstrainedLinConfig();

	/**
	 * Create the correct delta configuration
	 */
	//VectorConfig createConstrainedCorrectDelta();

	/**
	 * Create small example constrained factor graph
	 */
	//ConstrainedLinearFactorGraph createConstrainedLinearFactorGraph();

	/**
	 * Create small example constrained nonlinear factor graph
	 */
//	ConstrainedNonlinearFactorGraph<NonlinearFactor<VectorConfig>,VectorConfig>
//		createConstrainedNonlinearFactorGraph();
}
