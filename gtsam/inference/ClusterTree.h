/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ClusterTree.h
 * @date July 13, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <list>
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <gtsam/base/types.h>

namespace gtsam {

	/**
	 * A cluster-tree is associated with a factor graph and is defined as in Koller-Friedman:
	 * each node k represents a subset \f$ C_k \sub X \f$, and the tree is family preserving, in that
	 * each factor \f$ f_i \f$ is associated with a single cluster and \f$ scope(f_i) \sub C_k \f$.
	 * \nosubgrouping
	 */
	template <class FG>
	class ClusterTree {
	public:
		// Access to factor types
		typedef typename FG::KeyType KeyType;

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

			/// Construct empty clique
			Cluster() {}

			/** Create a node with a single frontal variable */
			template<typename Iterator>
			Cluster(const FG& fg, Index key, Iterator firstSeparator, Iterator lastSeparator);

      /** Create a node with several frontal variables */
      template<typename FRONTALIT, typename SEPARATORIT>
      Cluster(const FG& fg, FRONTALIT firstFrontal, FRONTALIT lastFrontal, SEPARATORIT firstSeparator, SEPARATORIT lastSeparator);

      /** Create a node with several frontal variables */
      template<typename FRONTALIT, typename SEPARATORIT>
      Cluster(FRONTALIT firstFrontal, FRONTALIT lastFrontal, SEPARATORIT firstSeparator, SEPARATORIT lastSeparator);

			/// print
			void print(const std::string& indent, const IndexFormatter& formatter = DefaultIndexFormatter) const;

			/// print the enire tree
			void printTree(const std::string& indent, const IndexFormatter& formatter = DefaultIndexFormatter) const;

			/// check equality
			bool equals(const Cluster& other) const;

			/// get a reference to the children
			const std::list<shared_ptr>& children() const { return children_; }

			/// add a child
			void addChild(shared_ptr child);

			/// get or set the parent
			weak_ptr& parent() { return parent_; }

		};

		/// @name Advanced Interface
		/// @{

		/// typedef for shared pointers to clusters
		typedef typename Cluster::shared_ptr sharedCluster;

		/// Root cluster
		sharedCluster root_;

	public:

		/// @}
		/// @name Standard Constructors
		/// @{

		/// constructor of empty tree
		ClusterTree() {}

		/// @}
		/// @name Standard Interface
		/// @{

		/// return the root cluster
		sharedCluster root() const { return root_; }

		/// @}
  	/// @name Testable
  	/// @{

		/// print the object
		void print(const std::string& str="", const IndexFormatter& formatter = DefaultIndexFormatter) const;

		/** check equality */
		bool equals(const ClusterTree<FG>& other, double tol = 1e-9) const;

		/// @}

	}; // ClusterTree

} // namespace gtsam

#include <gtsam/inference/ClusterTree-inl.h>
