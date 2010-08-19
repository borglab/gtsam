/*
 * EliminationTree.h
 * Created on: Feb 4, 2010
 * @Author: Kai Ni
 * @Author: Frank Dellaert
 * @brief: The elimination tree
 */

#pragma once

#include <set>
#include <gtsam/inference/IndexTable.h>
#include <gtsam/inference/ClusterTree.h>

namespace gtsam {

	/**
	 * An elimination tree (see Gilbert01bit) associated with a factor graph and an ordering
	 * is a cluster-tree where there is one node j for each variable, and the parent of each node
	 * corresponds to the first variable up the ordering in the Cholesky factor that j is connected to.
	 */
	template<class FG>
	class EliminationTree: public ClusterTree<FG> {

	public:

		// In an elimination tree, the clusters are called nodes
		typedef typename ClusterTree<FG>::Cluster Node;
		typedef typename Node::shared_ptr sharedNode;

		// we typedef the following handy list of ordered factor graphs
		typedef std::pair<Symbol, FG> NamedGraph;
		typedef std::list<NamedGraph> OrderedGraphs;

	private:

		/** Number of variables */
		size_t nrVariables_;

		/** Map from ordering index to Nodes */
		typedef std::vector<sharedNode> Nodes;
		Nodes nodes_;

		/**
		 * add a factor graph fragment with given frontal key into the tree. Assumes
		 * parent node was already added (will throw exception if not).
		 */
		void add(const FG& fg, const Symbol& key, const IndexTable<Symbol>& indexTable);

	public:

		/**
		 * Constructor variant 1: from an ordered list of factor graphs
		 * The list is supposed to be in elimination order, and for each
		 * eliminated variable a list of factors to be eliminated.
		 * This function assumes the input is correct (!) and will not check
		 * whether the factors refer only to the correct set of variables.
		 */
		EliminationTree(const OrderedGraphs& orderedGraphs);

		/**
		 * Constructor variant 2: given a factor graph and the elimination ordering
		 */
		EliminationTree(FG& fg, const Ordering& ordering);

	}; // EliminationTree

} // namespace gtsam
