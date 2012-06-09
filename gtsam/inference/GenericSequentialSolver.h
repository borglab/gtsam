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
 * @date    Oct 21, 2010
 */

#pragma once

#include <utility>
#include <boost/function.hpp>
#include <vector>
#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>

namespace gtsam { class VariableIndex; }
namespace gtsam { template<class FACTOR> class EliminationTree; }
namespace gtsam { template<class FACTOR> class FactorGraph; }
namespace gtsam { template<class CONDITIONAL> class BayesNet; }

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
	 * \nosubgrouping
	 */
	template<class FACTOR>
	class GenericSequentialSolver {

	protected:

		typedef boost::shared_ptr<FactorGraph<FACTOR> > sharedFactorGraph;

		typedef std::pair<
		        boost::shared_ptr<typename FACTOR::ConditionalType>,
		        boost::shared_ptr<FACTOR> > EliminationResult;
		typedef boost::function<EliminationResult(const FactorGraph<FACTOR>&, size_t)> Eliminate;

		/** Store the original factors for computing marginals
		 * TODO Frank says: really? Marginals should be computed from result.
		 */
		sharedFactorGraph factors_;

		/** Store column structure of the factor graph. Why? */
		boost::shared_ptr<VariableIndex> structure_;

		/** Elimination tree that performs elimination */
		boost::shared_ptr<EliminationTree<FACTOR> > eliminationTree_;

		/** concept checks */
		GTSAM_CONCEPT_TESTABLE_TYPE(FACTOR)
//		GTSAM_CONCEPT_TESTABLE_TYPE(EliminationTree)

	public:

		/// @name Standard Constructors
		/// @{

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
				const boost::shared_ptr<VariableIndex>& variableIndex);

		/// @}
		/// @name Testable
		/// @{

		/** Print to cout */
		void print(const std::string& name = "GenericSequentialSolver: ") const;

		/** Test whether is equal to another */
		bool equals(const GenericSequentialSolver& other, double tol = 1e-9) const;

		/// @}
		/// @name Standard Interface
		/// @{

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
		typename boost::shared_ptr<BayesNet<typename FACTOR::ConditionalType> > eliminate(Eliminate function) const;

		/**
		 * Compute the marginal joint over a set of variables, by integrating out
		 * all of the other variables.  Returns the result as a factor graph.
		 */
		typename FactorGraph<FACTOR>::shared_ptr jointFactorGraph(
				const std::vector<Index>& js, Eliminate function) const;

		/**
		 * Compute the marginal Gaussian density over a variable, by integrating out
		 * all of the other variables.  This function returns the result as a factor.
		 */
		typename boost::shared_ptr<FACTOR> marginalFactor(Index j, Eliminate function) const;

		/// @}

	}; // GenericSequentialSolver

} // namespace gtsam

#include <gtsam/inference/GenericSequentialSolver-inl.h>
