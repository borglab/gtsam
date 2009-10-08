/**
 * @file    smallExample.h
 * @brief   Create small example with two poses and one landmark
 * @brief   smallExample
 * @author  Carlos Nieto
 */

// \callgraph


#pragma once

#include <boost/shared_ptr.hpp>
//#include "ConstrainedNonlinearFactorGraph.h" // will be added back once design is solidified
//#include "ConstrainedLinearFactorGraph.h"
#include "NonlinearFactorGraph.h"
#include "ChordalBayesNet.h"
#include "LinearFactorGraph.h"
#include "FGConfig.h"

// \namespace

namespace gtsam {

	typedef NonlinearFactorGraph<FGConfig> ExampleNonlinearFactorGraph;

	/**
	 * Create small example for non-linear factor graph
	 */
	boost::shared_ptr<const ExampleNonlinearFactorGraph > sharedNonlinearFactorGraph();
	ExampleNonlinearFactorGraph createNonlinearFactorGraph();

	/**
	 * Create small example constrained factor graph
	 */
	//ConstrainedLinearFactorGraph createConstrainedLinearFactorGraph();

	/**
	 * Create small example constrained nonlinear factor graph
	 */
//	ConstrainedNonlinearFactorGraph<NonlinearFactor<FGConfig>,FGConfig>
//		createConstrainedNonlinearFactorGraph();

	/**
	 * Create configuration to go with it
	 * The ground truth configuration for the example above
	 */
	FGConfig createConfig();

	/**
	 * Create configuration for constrained example
	 * This is the ground truth version
	 */
	//FGConfig createConstrainedConfig();

	/**
	 * create a noisy configuration for a nonlinear factor graph
	 */
	boost::shared_ptr<const FGConfig> sharedNoisyConfig();
	FGConfig createNoisyConfig();

	/**
	 * Zero delta config
	 */
	FGConfig createZeroDelta();

	/**
	 * Delta config that, when added to noisyConfig, returns the ground truth
	 */
	FGConfig createCorrectDelta();

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
	 * Create a noisy configuration for linearization
	 */
	//FGConfig createConstrainedLinConfig();

	/**
	 * Create the correct delta configuration
	 */
	//FGConfig createConstrainedCorrectDelta();
}
