/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicFactorGraph.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/IndexFactor.h>

namespace gtsam { template<class FACTOR> class EliminationTree; }
namespace gtsam { template<class CONDITIONAL> class BayesNet; }
namespace gtsam { template<class CONDITIONAL, class CLIQUE> class BayesTree; }
namespace gtsam { class IndexConditional; }

namespace gtsam {

  typedef EliminationTree<IndexFactor> SymbolicEliminationTree;
	typedef BayesNet<IndexConditional> SymbolicBayesNet;
	typedef BayesTree<IndexConditional> SymbolicBayesTree;

	/** Symbolic IndexFactor Graph
	 *  \nosubgrouping
	 */
	class SymbolicFactorGraph: public FactorGraph<IndexFactor> {

	public:

		/// @name Standard Constructors
		/// @{

		/** Construct empty factor graph */
		SymbolicFactorGraph() {
		}

		/** Construct from a BayesNet */
		SymbolicFactorGraph(const SymbolicBayesNet& bayesNet);

    /** Construct from a BayesTree */
    SymbolicFactorGraph(const SymbolicBayesTree& bayesTree);

		/**
		 * Construct from a factor graph of any type
		 */
		template<class FACTOR>
		SymbolicFactorGraph(const FactorGraph<FACTOR>& fg);
		
		/** Eliminate the first \c n frontal variables, returning the resulting
		 * conditional and remaining factor graph - this is very inefficient for
		 * eliminating all variables, to do that use EliminationTree or
		 * JunctionTree.  Note that this version simply calls
		 * FactorGraph<IndexFactor>::eliminateFrontals with EliminateSymbolic
		 * as the eliminate function argument.
		 */
		std::pair<sharedConditional, SymbolicFactorGraph> eliminateFrontals(size_t nFrontals) const;

		/// @}
		/// @name Standard Interface
		/// @{

		/**
		 * Return the set of variables involved in the factors (computes a set
		 * union).
		 */
		FastSet<Index> keys() const;



		/// @}
		/// @name Advanced Interface
		/// @{

		/** Push back unary factor */
		void push_factor(Index key);

		/** Push back binary factor */
		void push_factor(Index key1, Index key2);

		/** Push back ternary factor */
		void push_factor(Index key1, Index key2, Index key3);

		/** Push back 4-way factor */
		void push_factor(Index key1, Index key2, Index key3, Index key4);

	};

	/** Create a combined joint factor (new style for EliminationTree). */
	IndexFactor::shared_ptr CombineSymbolic(const FactorGraph<IndexFactor>& factors,
		const FastMap<Index, std::vector<Index> >& variableSlots);

	/**
	 * CombineAndEliminate provides symbolic elimination.
	 * Combine and eliminate can also be called separately, but for this and
	 * derived classes calling them separately generally does extra work.
	 */
	std::pair<boost::shared_ptr<IndexConditional>, boost::shared_ptr<IndexFactor> >
	EliminateSymbolic(const FactorGraph<IndexFactor>&, size_t nrFrontals = 1);

	/// @}

	/** Template function implementation */
	template<class FACTOR>
	SymbolicFactorGraph::SymbolicFactorGraph(const FactorGraph<FACTOR>& fg) {
		for (size_t i = 0; i < fg.size(); i++) {
			if (fg[i]) {
				IndexFactor::shared_ptr factor(new IndexFactor(*fg[i]));
				push_back(factor);
			} else
				push_back(IndexFactor::shared_ptr());
		}
	}

} // namespace gtsam
