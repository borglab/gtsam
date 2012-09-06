/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    smallExample.h
 * @brief   Create small example with two poses and one landmark
 * @brief   smallExample
 * @author  Carlos Nieto
 */

// \callgraph


#pragma once

#include <tests/simulated2D.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/tuple/tuple.hpp>

namespace gtsam {
	namespace example {

		typedef NonlinearFactorGraph Graph;

		/**
		 * Create small example for non-linear factor graph
		 */
		boost::shared_ptr<const Graph> sharedNonlinearFactorGraph();
		Graph createNonlinearFactorGraph();

		/**
		 * Create values structure to go with it
		 * The ground truth values structure for the example above
		 */
		Values createValues();

		/** Vector Values equivalent */
		VectorValues createVectorValues();

		/**
		 * create a noisy values structure for a nonlinear factor graph
		 */
		boost::shared_ptr<const Values> sharedNoisyValues();
		Values createNoisyValues();

		/**
		 * Zero delta config
		 */
		VectorValues createZeroDelta(const Ordering& ordering);

		/**
		 * Delta config that, when added to noisyValues, returns the ground truth
		 */
		VectorValues createCorrectDelta(const Ordering& ordering);

		/**
		 * create a linear factor graph
		 * The non-linear graph above evaluated at NoisyValues
		 */
		GaussianFactorGraph createGaussianFactorGraph(const Ordering& ordering);

		/**
		 * create small Chordal Bayes Net x <- y
		 */
		GaussianBayesNet createSmallGaussianBayesNet();

		/**
		 * Create really non-linear factor graph (cos/sin)
		 */
		boost::shared_ptr<const Graph>
		sharedReallyNonlinearFactorGraph();
		Graph createReallyNonlinearFactorGraph();

		/**
		 * Create a full nonlinear smoother
		 * @param T number of time-steps
		 */
		std::pair<Graph, Values> createNonlinearSmoother(int T);

		/**
		 * Create a Kalman smoother by linearizing a non-linear factor graph
		 * @param T number of time-steps
		 */
		std::pair<FactorGraph<GaussianFactor>, Ordering> createSmoother(int T, boost::optional<Ordering> ordering = boost::none);

		/* ******************************************************* */
		// Linear Constrained Examples
		/* ******************************************************* */

		/**
		 * Creates a simple constrained graph with one linear factor and
		 * one binary equality constraint that sets x = y
		 */
		GaussianFactorGraph createSimpleConstraintGraph();
		VectorValues createSimpleConstraintValues();

		/**
		 * Creates a simple constrained graph with one linear factor and
		 * one binary constraint.
		 */
		GaussianFactorGraph createSingleConstraintGraph();
		VectorValues createSingleConstraintValues();

		/**
		 * Creates a constrained graph with a linear factor and two
		 * binary constraints that share a node
		 */
		GaussianFactorGraph createMultiConstraintGraph();
		VectorValues createMultiConstraintValues();

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
		boost::tuple<GaussianFactorGraph, VectorValues> planarGraph(size_t N);

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
		std::pair<GaussianFactorGraph, GaussianFactorGraph > splitOffPlanarTree(
				size_t N, const GaussianFactorGraph& original);

	} // example
} // gtsam
