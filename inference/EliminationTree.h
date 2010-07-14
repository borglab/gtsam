/*
 * EliminationTree.h
 * Created on: Feb 4, 2010
 * @Author: Kai Ni
 * @Author: Frank Dellaert
 * @brief: The elimination tree
 */

#pragma once

#include <set>
#include "ClusterTree.h"

namespace gtsam {

	/**
	 * An elimination tree (see Gilbert01bit) associated with a factorg raph and an ordering
	 * is a cluster-tree where there is one node j for each variable, and the parent of each node
	 * corresponds to the first variable in the ordering that variable j is connected to.
	 */
	template<class FG>
	class EliminationTree: public ClusterTree<FG> {

	public:

		// In a junction tree each cluster is associated with a clique
		typedef typename ClusterTree<FG>::Cluster Node;
		typedef typename Node::shared_ptr sharedNode;

	public:
		// constructor
		EliminationTree() {
		}

		// constructor given a factor graph and the elimination ordering
		EliminationTree(FG& fg, const Ordering& ordering);

	}; // EliminationTree

} // namespace gtsam
