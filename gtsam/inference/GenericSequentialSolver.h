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

#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/BayesNet.h>

#include <utility>

namespace gtsam {

	template<class FACTOR>
	class GenericSequentialSolver : public Testable<GenericSequentialSolver<FACTOR> > {

	protected:

		// Store the original factors for computing marginals
		typename FactorGraph<FACTOR>::shared_ptr factors_;

		// Column structure of the factor graph
		VariableIndex::shared_ptr structure_;

		// Elimination tree that performs elimination.
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
				const typename FactorGraph<FACTOR>::shared_ptr& factorGraph,
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
		void replaceFactors(
				const typename FactorGraph<FACTOR>::shared_ptr& factorGraph);

		/**
		 * Eliminate the factor graph sequentially.  Uses a column elimination tree
		 * to recursively eliminate.
		 */
		typename BayesNet<typename FACTOR::ConditionalType>::shared_ptr
				eliminate() const;

		/**
		 * Compute the marginal joint over a set of variables, by integrating out
		 * all of the other variables.  This function returns the result as a factor
		 * graph.
		 */
		typename FactorGraph<FACTOR>::shared_ptr jointFactorGraph(
				const std::vector<Index>& js) const;

		/**
		 * Compute the marginal Gaussian density over a variable, by integrating out
		 * all of the other variables.  This function returns the result as a factor.
		 */
		typename FACTOR::shared_ptr marginalFactor(Index j) const;

	}; // GenericSequentialSolver

} // namespace gtsam

