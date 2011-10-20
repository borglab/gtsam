/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericSequentialSolver.h
 * @brief   generic sequential elimination
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#pragma once

#include <utility>

#include <gtsam/inference/EliminationTree.h>

namespace gtsam {

	/**
	 * This solver implements sequential variable elimination for factor graphs.
	 * Underlying this is a column elimination tree, see Gilbert 2001 BIT.
	 *
	 * The elimination ordering is "baked in" to the variable indices at this
	 * stage, i.e. elimination proceeds in order from '0'.
	 *
	 * This is not the most efficient algorithm we provide, most efficient is the
	 * MultifrontalSolver, which examines and uses the clique structure.
	 * However, sequential variable elimination is easier to understand so this is a good
	 * starting point to learn about these algorithms and our implementation.
	 * Additionally, the first step of MFQR is symbolic sequential elimination.
	 */
	template<class FACTOR>
	class GenericSequentialSolver {

	protected:

		typedef typename FactorGraph<FACTOR>::shared_ptr sharedFactorGraph;

		/** Store the original factors for computing marginals
		 * TODO Frank says: really? Marginals should be computed from result.
		 */
		sharedFactorGraph factors_;

		/** Store column structure of the factor graph. Why? */
		VariableIndex::shared_ptr structure_;

		/** Elimination tree that performs elimination */
		typename EliminationTree<FACTOR>::shared_ptr eliminationTree_;

	public:

		/**
		 * Construct the solver for a factor graph.  This builds the elimination
		 * tree, which already does some of the work of elimination.
		 */
		GenericSequentialSolver(const FactorGraph<FACTOR>& factorGraph);

		/**
		 * Construct the solver with a shared pointer to a factor graph and to a
		 * VariableIndex.  The solver will store these pointers, so this constructor
		 * is the fastest.
		 */
		GenericSequentialSolver(
				const sharedFactorGraph& factorGraph,
				const VariableIndex::shared_ptr& variableIndex);

		/** Print to cout */
		void print(const std::string& name = "GenericSequentialSolver: ") const;

		/** Test whether is equal to another */
		bool equals(const GenericSequentialSolver& other, double tol = 1e-9) const;

		/**
		 * Replace the factor graph with a new one having the same structure.  The
		 * This function can be used if the numerical part of the factors changes,
		 * such as during relinearization or adjusting of noise models.
		 */
		void replaceFactors(const sharedFactorGraph& factorGraph);

		/**
		 * Eliminate the factor graph sequentially.  Uses a column elimination tree
		 * to recursively eliminate.
		 */
		typename BayesNet<typename FACTOR::ConditionalType>::shared_ptr
		eliminate(typename EliminationTree<FACTOR>::Eliminate function) const;

		/**
		 * Compute the marginal joint over a set of variables, by integrating out
		 * all of the other variables.  Returns the result as a factor graph.
		 */
		typename FactorGraph<FACTOR>::shared_ptr jointFactorGraph(
				const std::vector<Index>& js,
				typename EliminationTree<FACTOR>::Eliminate function) const;

		/**
		 * Compute the marginal Gaussian density over a variable, by integrating out
		 * all of the other variables.  This function returns the result as a factor.
		 */
		typename FACTOR::shared_ptr marginalFactor(Index j,
				typename EliminationTree<FACTOR>::Eliminate function) const;

	}; // GenericSequentialSolver

} // namespace gtsam

