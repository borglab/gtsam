/*
 * JunctionTree.h
 * Created on: Feb 4, 2010
 * @Author: Kai Ni
 * @Author: Frank Dellaert
 * @brief: The junction tree
 */

#pragma once

#include <set>
#include <boost/shared_ptr.hpp>
#include "BayesTree.h"
#include "ClusterTree.h"
#include "SymbolicConditional.h"

namespace gtsam {

	/**
	 * A junction tree (or clique-tree) is a cluster-tree where each node k represents a
	 * clique (maximal fully connected subset) of an associated chordal graph, such as a
	 * chordal Bayes net resulting from elimination. In GTSAM the BayesTree is used to
	 * represent the clique tree associated with a Bayes net, and the JunctionTree is
	 * used to collect the factors associated with each clique during the elimination process.
	 */
	template<class FG>
	class JunctionTree: public ClusterTree<FG> {

	public:

		// In a junction tree each cluster is associated with a clique
		typedef typename ClusterTree<FG>::Cluster Clique;
		typedef typename Clique::shared_ptr sharedClique;

		// And we will frequently refer to a symbolic Bayes tree
		typedef BayesTree<SymbolicConditional> SymbolicBayesTree;

	private:
		// distribute the factors along the cluster tree
		sharedClique distributeFactors(FG& fg,
				const SymbolicBayesTree::sharedClique clique);

		// utility function called by eliminate
		template<class Conditional>
		std::pair<FG, BayesTree<Conditional> > eliminateOneClique(sharedClique fg_);

	public:
		// constructor
		JunctionTree() {
		}

		// constructor given a factor graph and the elimination ordering
		JunctionTree(FG& fg, const Ordering& ordering);

		// eliminate the factors in the subgraphs
		template<class Conditional>
		BayesTree<Conditional> eliminate();

	}; // JunctionTree

} // namespace gtsam
