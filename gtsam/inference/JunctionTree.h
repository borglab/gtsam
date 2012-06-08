/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file JunctionTree.h
 * @date Feb 4, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: The junction tree
 */

#pragma once

#include <set>
#include <vector>
#include <gtsam/base/FastList.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/ClusterTree.h>
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/inference/VariableIndex.h>

namespace gtsam {

	/**
	 * A ClusterTree, i.e., a set of variable clusters with factors, arranged in a tree, with
	 * the additional property that it represents the clique tree associated with a Bayes net.
	 *
	 * In GTSAM a junction tree is an intermediate data structure in multifrontal
	 * variable elimination.  Each node is a cluster of factors, along with a
	 * clique of variables that are eliminated all at once. In detail, every node k represents
	 * a clique (maximal fully connected subset) of an associated chordal graph, such as a
	 * chordal Bayes net resulting from elimination.
	 *
	 * The difference with the BayesTree is that a JunctionTree stores factors, whereas a
	 * BayesTree stores conditionals, that are the product of eliminating the factors in the
	 * corresponding JunctionTree cliques.
	 *
	 * The tree structure and elimination method are exactly analagous to the EliminationTree,
	 * except that in the JunctionTree, at each node multiple variables are eliminated at a time.
	 *
	 *
	 * \addtogroup Multifrontal
	 * \nosubgrouping
	 */
	template<class FG, class BTCLIQUE=typename BayesTree<typename FG::FactorType::ConditionalType>::Clique>
	class JunctionTree: public ClusterTree<FG> {

	public:

		/// In a junction tree each cluster is associated with a clique
		typedef typename ClusterTree<FG>::Cluster Clique;
		typedef typename Clique::shared_ptr sharedClique; ///< Shared pointer to a clique

		/// The BayesTree type produced by elimination
		typedef BTCLIQUE BTClique;

		/// Shared pointer to this class
		typedef boost::shared_ptr<JunctionTree<FG> > shared_ptr;

		/// We will frequently refer to a symbolic Bayes tree, used to find the clique structure
		typedef gtsam::BayesTree<IndexConditional> SymbolicBayesTree;

	private:

		/// @name Advanced Interface
		/// @{

		/// distribute the factors along the cluster tree
		sharedClique distributeFactors(const FG& fg,
				const SymbolicBayesTree::sharedClique& clique);

		/// distribute the factors along the cluster tree
    sharedClique distributeFactors(const FG& fg, const std::vector<FastList<size_t> >& targets,
        const SymbolicBayesTree::sharedClique& clique);

		/// recursive elimination function
		std::pair<typename BTClique::shared_ptr, typename FG::sharedFactor>
		eliminateOneClique(typename FG::Eliminate function,
				const boost::shared_ptr<const Clique>& clique) const;

		/// internal constructor
		void construct(const FG& fg, const VariableIndex& variableIndex);

		/// @}

	public:

		/// @name Standard Constructors
		/// @{

		/** Default constructor */
		JunctionTree() {}

		/** Named constructor to build the junction tree of a factor graph.  Note
	   * that this has to compute the column structure as a VariableIndex, so if you
	   * already have this precomputed, use the JunctionTree(const FG&, const VariableIndex&)
	   * constructor instead.
	   * @param factorGraph The factor graph for which to build the elimination tree
	   */
		JunctionTree(const FG& factorGraph);

		/** Construct from a factor graph and pre-computed variable index.
		 * @param fg The factor graph for which to build the junction tree
		 * @param structure The set of factors involving each variable.  If this is not
		 * precomputed, you can call the JunctionTree(const FG&)
		 * constructor instead.
		 */
		JunctionTree(const FG& fg, const VariableIndex& variableIndex);

		/// @}
		/// @name Standard Interface
		/// @{

		/** Eliminate the factors in the subgraphs to produce a BayesTree.
		 * @param function The function used to eliminate, see the namespace functions
		 * in GaussianFactorGraph.h
		 * @return The BayesTree resulting from elimination
		 */
		typename BTClique::shared_ptr eliminate(typename FG::Eliminate function) const;

		/// @}

	}; // JunctionTree

} // namespace gtsam

#include <gtsam/inference/JunctionTree-inl.h>
