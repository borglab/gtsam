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

	/** A clique in a Bayes tree consisting of frontal nodes and conditionals */
	template<class Conditional>
	class Front: Testable<Front<Conditional> > {
	private:
		typedef boost::shared_ptr<Conditional> conditional_ptr;
		std::list<conditional_ptr> conditionals_; /** conditionals */
		std::list<std::string> separator_; /** separator keys */
	public:

		/** constructor */
		Front(const conditional_ptr& conditional);

		/** print */
		void print(const std::string& s = "") const;

		/** check equality */
		bool equals(const Front<Conditional>& other, double tol = 1e-9) const;

		/** add a frontal node */
		void add(const conditional_ptr& conditional);

		/** return size of the clique */
		inline size_t size() const {return conditionals_.size() + separator_.size();}
	};

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

		/** A Node in the tree is a Front with tree connectivity */
		struct Node : public Front<Conditional> {
			typedef boost::shared_ptr<Node> shared_ptr;
			shared_ptr parent_;
			std::list<shared_ptr> children_;

			Node(const boost::shared_ptr<Conditional>& conditional):Front<Conditional>(conditional) {}

			/** print this node and entire subtree below it*/
			void printTree(const std::string& indent) const {
				print(indent);
				BOOST_FOREACH(shared_ptr child, children_)
					child->printTree(indent+"  ");
			}
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
		const Front<Conditional>& root() const {return *(nodes_[0]);}

	}; // BayesTree

} /// namespace gtsam
