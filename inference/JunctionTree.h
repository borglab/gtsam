/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * JunctionTree.h
 * Created on: Feb 4, 2010
 * @Author: Kai Ni
 * @Author: Frank Dellaert
 * @brief: The junction tree
 */

#pragma once

#include <set>
#include <vector>
#include <list>
#include <boost/shared_ptr.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/ClusterTree.h>
#include <gtsam/inference/Conditional.h>

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

		typedef class BayesTree<typename FG::factor_type::Conditional> BayesTree;

		// And we will frequently refer to a symbolic Bayes tree
		typedef gtsam::BayesTree<Conditional> SymbolicBayesTree;

	private:
		// distribute the factors along the cluster tree
		sharedClique distributeFactors(const FG& fg,
				const SymbolicBayesTree::sharedClique& clique);

		// distribute the factors along the cluster tree
    sharedClique distributeFactors(const FG& fg, const std::vector<std::list<size_t,boost::fast_pool_allocator<size_t> > >& targets,
        const SymbolicBayesTree::sharedClique& clique);

		// recursive elimination function
		std::pair<typename BayesTree::sharedClique, typename FG::sharedFactor>
		eliminateOneClique(const boost::shared_ptr<const Clique>& clique) const;

	public:
		// constructor
		JunctionTree() {
		}

		// constructor given a factor graph and the elimination ordering
		JunctionTree(const FG& fg);

		// eliminate the factors in the subgraphs
		typename BayesTree::sharedClique eliminate() const;

	}; // JunctionTree

} // namespace gtsam
