/*
 * ClusterTree.h
 * Created on: July 13, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "Ordering.h"

namespace gtsam {

	/**
	 * A cluster-tree is associated with a factor graph and is defined as in Koller-Friedman:
	 * each node k represents a subset C_k \sub X, and the tree is family preserving, in that
	 * each factor f_i is associated with a single cluster and scope(f_i) \sub C_k.
	 */
	template <class FG>
	class ClusterTree : public Testable<ClusterTree<FG> > {

	protected:

		// the class for subgraphs that also include the pointers to the parents and two children
		struct Cluster : public FG {

			typedef typename boost::shared_ptr<Cluster> shared_ptr;

			Ordering frontal_;                   // the frontal variables
			Unordered separator_;                // the separator variables
			shared_ptr parent_;                  // the parent cluster
			std::vector<shared_ptr> children_;   // the child clusters

			// Construct empty clique
			Cluster() {}

			/* Create a node with a single frontal variable */
			Cluster(const FG& fg, const Symbol& key);

			// print the object
			void print(const std::string& indent) const;
			void printTree(const std::string& indent) const;

			// check equality
			bool equals(const Cluster& other) const;
		};

		// typedef for shared pointers to clusters
		typedef typename Cluster::shared_ptr sharedCluster;

		// Root cluster
		sharedCluster root_;

	public:
		// constructor of empty tree
		ClusterTree() {}

		// return the root cluster
		sharedCluster root() const { return root_; }

		// print the object
		void print(const std::string& str) const {
			std::cout << str << std::endl;
			if (root_.get()) root_->printTree("");
		}

		/** check equality */
		bool equals(const ClusterTree<FG>& other, double tol = 1e-9) const;

	}; // ClusterTree

} // namespace gtsam
