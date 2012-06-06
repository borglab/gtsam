/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteSequentialSolver.h
 * @date Feb 16, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/inference/GenericSequentialSolver.h>
#include <boost/shared_ptr.hpp>

namespace gtsam {
	// The base class provides all of the needed functionality

	class DiscreteSequentialSolver: public GenericSequentialSolver<DiscreteFactor> {

	protected:
		typedef GenericSequentialSolver<DiscreteFactor> Base;
		typedef boost::shared_ptr<const DiscreteSequentialSolver> shared_ptr;

	public:

		/**
		 * The problem we are trying to solve (SUM or MPE).
		 */
		typedef enum {
			BEL, // Belief updating (or conditional updating)
			MPE, // Most-Probable-Explanation
			MAP
		// Maximum A Posteriori hypothesis
		} ProblemType;

		/**
		 * Construct the solver for a factor graph.  This builds the elimination
		 * tree, which already does some of the work of elimination.
		 */
		DiscreteSequentialSolver(const FactorGraph<DiscreteFactor>& factorGraph) :
			Base(factorGraph) {
		}

		/**
		 * Construct the solver with a shared pointer to a factor graph and to a
		 * VariableIndex.  The solver will store these pointers, so this constructor
		 * is the fastest.
		 */
		DiscreteSequentialSolver(
				const FactorGraph<DiscreteFactor>::shared_ptr& factorGraph,
				const VariableIndex::shared_ptr& variableIndex) :
			Base(factorGraph, variableIndex) {
		}

		const EliminationTree<DiscreteFactor>& eliminationTree() const {
			return *eliminationTree_;
		}

		/**
		 * Eliminate the factor graph sequentially.  Uses a column elimination tree
		 * to recursively eliminate.
		 */
		BayesNet<DiscreteConditional>::shared_ptr eliminate() const {
			return Base::eliminate(&EliminateDiscrete);
		}

		/**
		 * Compute the marginal joint over a set of variables, by integrating out
		 * all of the other variables.  This function returns the result as a factor
		 * graph.
		 */
		DiscreteFactorGraph::shared_ptr jointFactorGraph(
				const std::vector<Index>& js) const {
			DiscreteFactorGraph::shared_ptr results(new DiscreteFactorGraph(
					*Base::jointFactorGraph(js, &EliminateDiscrete)));
			return results;
		}

		/**
		 * Compute the marginal density over a variable, by integrating out
		 * all of the other variables.  This function returns the result as a factor.
		 */
		DiscreteFactor::shared_ptr marginalFactor(Index j) const {
			return Base::marginalFactor(j, &EliminateDiscrete);
		}

		/**
		 * Compute the marginal density over a variable, by integrating out
		 * all of the other variables. This function returns the result as a
		 * Vector of the probability values.
		 */
		Vector marginalProbabilities(const DiscreteKey& key) const;

		/**
		 * Compute the MPE solution of the DiscreteFactorGraph.  This
		 * eliminates to create a BayesNet and then back-substitutes this BayesNet to
		 * obtain the solution.
		 */
		DiscreteFactor::sharedValues optimize() const;

	};

} // gtsam
