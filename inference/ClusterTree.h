/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * ClusterTree.h
 * Created on: July 13, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <list>
#include <vector>
#include <iostream>

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
		class Cluster : public FG {
		public:
			typedef typename boost::shared_ptr<Cluster> shared_ptr;
			typedef typename boost::weak_ptr<Cluster> weak_ptr;

      const std::vector<Index> frontal;                   // the frontal variables
      const std::vector<Index> separator;                // the separator variables

		protected:

			weak_ptr parent_;                      // the parent cluster
			std::list<shared_ptr> children_;     // the child clusters
			const typename FG::sharedFactor eliminated_; // the eliminated factor to pass on to the parent

		public:

			// Construct empty clique
			Cluster() {}

			/* Create a node with a single frontal variable */
			template<typename Iterator>
			Cluster(const FG& fg, Index key, Iterator firstSeparator, Iterator lastSeparator);

      /* Create a node with several frontal variables */
      template<typename FrontalIt, typename SeparatorIt>
      Cluster(const FG& fg, FrontalIt firstFrontal, FrontalIt lastFrontal, SeparatorIt firstSeparator, SeparatorIt lastSeparator);

      /* Create a node with several frontal variables */
      template<typename FrontalIt, typename SeparatorIt>
      Cluster(FrontalIt firstFrontal, FrontalIt lastFrontal, SeparatorIt firstSeparator, SeparatorIt lastSeparator);

			// print the object
			void print(const std::string& indent) const;
			void printTree(const std::string& indent) const;

			// check equality
			bool equals(const Cluster& other) const;

			// get or set the parent
			weak_ptr& parent() { return parent_; }

			// get a reference to the children
			const std::list<shared_ptr>& children() const { return children_; }

			// add a child
			void addChild(shared_ptr child);
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
			if (root_) root_->printTree("");
		}

		/** check equality */
		bool equals(const ClusterTree<FG>& other, double tol = 1e-9) const;

	}; // ClusterTree

} // namespace gtsam
