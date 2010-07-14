/*
 * ClusterTree.h
 * Created on: July 13, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <set>
#include <boost/shared_ptr.hpp>

namespace gtsam {

	/**
	 * A cluster-tree is associated with a factor graph and is defined as in Koller-Friedman:
	 * each node k represents a subset C_k \sub X, and the tree is family preserving, in that
	 * each factor f_i is associated with a single cluster and scope(f_i) \sub C_k.
	 */
	template <class FG>
	class ClusterTree : public Testable<ClusterTree<FG> > {

	public:

		// the class for subgraphs that also include the pointers to the parents and two children
		class Cluster : public FG {

		public:

			typedef typename boost::shared_ptr<Cluster> shared_ptr;

		/* commented private out to make compile but needs to be addressed */

			shared_ptr parent_;                  // the parent subgraph node
			std::vector<shared_ptr> children_;   // the child clusters
			Ordering frontal_;                   // the frontal variables
			Unordered separator_;                // the separator variables

		public:

			// empty constructor
			Cluster() {}

			// constructor with all the information
			Cluster(const FG& fgLocal, const Ordering& frontal, const Unordered& separator,
					 const shared_ptr& parent)
				: frontal_(frontal), separator_(separator), FG(fgLocal), parent_(parent) {}

			// constructor for an empty graph
			Cluster(const Ordering& frontal, const Unordered& separator, const shared_ptr& parent)
				: frontal_(frontal), separator_(separator), parent_(parent) {}

			// return the members
			const Ordering& frontal() const            { return frontal_;}
			const Unordered& separator() const         { return separator_;}
			const std::vector<shared_ptr>& children()  { return children_; }

			// add a child node
			void addChild(const shared_ptr& child) { children_.push_back(child); }

			// print the object
			void print(const std::string& indent) const;
			void printTree(const std::string& indent) const;

			// check equality
			bool equals(const Cluster& other) const;
		};

		// typedef for shared pointers to clusters
		typedef typename Cluster::shared_ptr sharedCluster;

	protected:
		// Root cluster
		sharedCluster root_;

	public:
		// constructor
		ClusterTree() {}

		// constructor given a factor graph and the elimination ordering
		ClusterTree(FG& fg, const Ordering& ordering);

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
