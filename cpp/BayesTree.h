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
		typedef std::pair<std::string,conditional_ptr> NamedConditional;

	private:

		/** A Node in the tree is an incomplete Bayes net: the variables
		 * in the Bayes net are the frontal nodes, and the variables conditioned
		 * on is the separator. We also have pointers up and down the tree.
		 */
		struct Node : public BayesNet<Conditional> {
			typedef boost::shared_ptr<Node> shared_ptr;
			shared_ptr parent_;
			std::list<std::string> separator_; /** separator keys */
			std::list<shared_ptr> children_;

			//* Constructor */
			Node(const boost::shared_ptr<Conditional>& conditional);

			/** The size *includes* the separator */
			size_t size() const { return this->conditionals_.size() + separator_.size(); }

			/** print this node */
			void print(const std::string& s="Bayes tree node") const;

			/** print this node and entire subtree below it*/
			void printTree(const std::string& indent) const;
		};

		/** vector of Nodes */
		typedef boost::shared_ptr<Node> node_ptr;
		typedef std::vector<node_ptr> Nodes;
		Nodes nodes_;

		/** Map from keys to Node index */
		typedef std::map<std::string, int> NodeMap;
		NodeMap nodeMap_;

	public:

		/** Create an empty Bayes Tree */
		BayesTree();

		/** Create a Bayes Tree from a Bayes Net */
		BayesTree(const BayesNet<Conditional>& bayesNet, bool verbose=false);

		/** Destructor */
		virtual ~BayesTree() {}

		/** print */
		void print(const std::string& s = "") const;

		/** check equality */
		bool equals(const BayesTree<Conditional>& other, double tol = 1e-9) const;

		/** insert a new conditional */
		void insert(const boost::shared_ptr<Conditional>& conditional, bool verbose=false);

			/** number of cliques */
		inline size_t size() const { return nodes_.size();}

		/** return root clique */
		const BayesNet<Conditional>& root() const {return *(nodes_[0]);}

	}; // BayesTree

} /// namespace gtsam
