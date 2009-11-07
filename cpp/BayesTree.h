/**
 * @file    BayesTree
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <map>
#include <list>
#include <vector>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

#include "Testable.h"
#include "BayesNet.h"

namespace gtsam {

	/**
	 * Bayes tree
	 * Templated on the Conditional class, the type of node in the underlying Bayes chain.
	 * This could be a ConditionalProbabilityTable, a ConditionalGaussian, or a SymbolicConditional
	 */
	template<class Conditional>
	class BayesTree: public Testable<BayesTree<Conditional> > {

	public:

		typedef boost::shared_ptr<Conditional> conditional_ptr;

		/** A Node in the tree is an incomplete Bayes net: the variables
		 * in the Bayes net are the frontal nodes, and the variables conditioned
		 * on is the separator. We also have pointers up and down the tree.
		 */
		struct Node: public BayesNet<Conditional> {

			typedef boost::shared_ptr<Node> shared_ptr;
			shared_ptr parent_;
			std::list<std::string> separator_; /** separator keys */
			std::list<shared_ptr> children_;

			//* Constructor */
			Node(const conditional_ptr& conditional);

			/** The size *includes* the separator */
			size_t size() const {
				return this->conditionals_.size() + separator_.size();
			}

			/** print this node */
			void print(const std::string& s = "Bayes tree node") const;

			/** print this node and entire subtree below it*/
			void printTree(const std::string& indent) const;
		};

		typedef boost::shared_ptr<Node> node_ptr;

	private:

		/** Map from keys to Node */
		typedef std::map<std::string, node_ptr> Nodes;
		Nodes nodes_;

		/** Roor clique */
		node_ptr root_;

		/** add a clique */
		node_ptr addClique(const conditional_ptr& conditional,
				node_ptr parent_clique = node_ptr());

	public:

		/** Create an empty Bayes Tree */
		BayesTree();

		/** Create a Bayes Tree from a Bayes Net */
		BayesTree(const BayesNet<Conditional>& bayesNet);

		/** Destructor */
		virtual ~BayesTree() {
		}

		/** print */
		void print(const std::string& s = "") const;

		/** check equality */
		bool equals(const BayesTree<Conditional>& other, double tol = 1e-9) const;

		/** insert a new conditional */
		void insert(const conditional_ptr& conditional);

		/** number of cliques */
		inline size_t size() const {
			return nodes_.size();
		}

		/** return root clique */
		node_ptr root() const {
			return root_;
		}

		/** find the clique to which key belongs */
		node_ptr operator[](const std::string& key) const {
			typename Nodes::const_iterator it = nodes_.find(key);
			if (it == nodes_.end()) throw(std::invalid_argument(
					"BayesTree::operator['" + key + "'): key not found"));
			node_ptr clique = it->second;
			return clique;
		}

		/** return marginal on any variable */
		template<class Factor>
		conditional_ptr marginal(const std::string& key) const;

	}; // BayesTree

} /// namespace gtsam
