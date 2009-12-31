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
	GaussianFactorGraph createGaussianFactorGraph();

	/**
	 * create small Chordal Bayes Net x <- y
	 */
	GaussianBayesNet createSmallGaussianBayesNet();

	/**
	 * Create really non-linear factor graph (cos/sin)
	 */
	boost::shared_ptr<const ExampleNonlinearFactorGraph> sharedReallyNonlinearFactorGraph();
	ExampleNonlinearFactorGraph createReallyNonlinearFactorGraph();

	/**
	 * Create a full nonlinear smoother
	 * @param T number of time-steps
	 */
	std::pair<ExampleNonlinearFactorGraph, VectorConfig> createNonlinearSmoother(int T);

	/**
	 * Create a Kalman smoother by linearizing a non-linear factor graph
	 * @param T number of time-steps
	 */
	GaussianFactorGraph createSmoother(int T);


	/* ******************************************************* */
	// Constrained Examples
	/* ******************************************************* */

	/**
	 * Creates a simple constrained graph with one linear factor and
	 * one binary equality constraint that sets x = y
	 */
	GaussianFactorGraph createSimpleConstraintGraph();
	VectorConfig createSimpleConstraintConfig();

	/**
	 * Creates a simple constrained graph with one linear factor and
	 * one binary constraint.
	 */
	GaussianFactorGraph createSingleConstraintGraph();
	VectorConfig createSingleConstraintConfig();

	/**
	 * Creates a constrained graph with a linear factor and two
	 * binary constraints that share a node
	 */
	GaussianFactorGraph createMultiConstraintGraph();
	VectorConfig createMultiConstraintConfig();

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
	//GaussianFactorGraph createConstrainedGaussianFactorGraph();

	/**
	 * Create small example constrained nonlinear factor graph
	 */
//	ConstrainedNonlinearFactorGraph<NonlinearFactor<VectorConfig>,VectorConfig>
//		createConstrainedNonlinearFactorGraph();

	/* ******************************************************* */
	// Planar graph with easy subtree for SubgraphPreconditioner
	/* ******************************************************* */

	/*
	 * Create factor graph with N^2 nodes, for example for N=3
	 *  x13-x23-x33
	 *   |   |   |
	 *  x12-x22-x32
	 *   |   |   |
	 * -x11-x21-x31
	 * with x11 clamped at (1,1), and others related by 2D odometry.
	 */
	std::pair<GaussianFactorGraph, VectorConfig> planarGraph(size_t N);

	/*
	 * Create canonical ordering for planar graph that also works for tree
	 * With x11 the root, e.g. for N=3
	 * x33 x23 x13 x32 x22 x12 x31 x21 x11
	 */
	Ordering planarOrdering(size_t N);

	/*
	 * Split graph into tree and loop closing constraints, e.g., with N=3
	 *  x13-x23-x33
	 *   |
	 *  x12-x22-x32
	 *   |
	 * -x11-x21-x31
	 */
	std::pair<GaussianFactorGraph, GaussianFactorGraph> splitOffPlanarTree(size_t N,
			const GaussianFactorGraph& original);

} // gtsam
