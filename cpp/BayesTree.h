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
#include <stdexcept>

#include "Testable.h"
#include "FactorGraph.h"
#include "BayesNet.h"
#include "Key.h"
#include "IndexTable.h"

namespace gtsam {

	/**
	 * Bayes tree
	 * Templated on the Conditional class, the type of node in the underlying Bayes chain.
	 * This could be a ConditionalProbabilityTable, a GaussianConditional, or a SymbolicConditional
	 */
	template<class Conditional>
	class BayesTree: public Testable<BayesTree<Conditional> > {

	public:

		typedef boost::shared_ptr<Conditional> sharedConditional;
		typedef boost::shared_ptr<BayesNet<Conditional> > sharedBayesNet;

		/** A Clique in the tree is an incomplete Bayes net: the variables
		 * in the Bayes net are the frontal nodes, and the variables conditioned
		 * on is the separator. We also have pointers up and down the tree.
		 */
		struct Clique: public BayesNet<Conditional> {

			typedef typename boost::shared_ptr<Clique> shared_ptr;
			shared_ptr parent_;
			std::list<shared_ptr> children_;
			std::list<Symbol> separator_; /** separator keys */

			//* Constructor */
			Clique(const sharedConditional& conditional);

			/** return keys in frontal:separator order */
			Ordering keys() const;

			/** print this node */
			void print(const std::string& s = "") const;

			/** The size *includes* the separator */
			size_t size() const {
				return this->conditionals_.size() + separator_.size();
			}

			/** is this the root of a Bayes tree ? */
			inline bool isRoot() const { return parent_==NULL;}

			/** The size of subtree rooted at this clique, i.e., nr of Cliques */
			size_t treeSize() const;

			/** print this node and entire subtree below it */
			void printTree(const std::string& indent="") const;

			/** return the conditional P(S|Root) on the separator given the root */
			// TODO: create a cached version
			template<class Factor>
			BayesNet<Conditional> shortcut(shared_ptr root);

			/** return the marginal P(C) of the clique */
			template<class Factor>
			FactorGraph<Factor> marginal(shared_ptr root);

			/** return the joint P(C1,C2), where C1==this. TODO: not a method? */
			template<class Factor>
			FactorGraph<Factor> joint(shared_ptr C2, shared_ptr root);
		};

		// typedef for shared pointers to cliques
		typedef boost::shared_ptr<Clique> sharedClique;

		// A convenience class for a list of shared cliques
		struct Cliques : public std::list<sharedClique>, public Testable<Cliques> {
			void print(const std::string& s = "Cliques") const;
			bool equals(const Cliques& other, double tol = 1e-9) const;
		};

	private:

		/** Map from keys to Clique */
		typedef std::map<Symbol, sharedClique> Nodes;
		Nodes nodes_;

		/** Root clique */
		sharedClique root_;

		/** add a clique */
		sharedClique addClique(const sharedConditional& conditional,
				sharedClique parent_clique = sharedClique());

	protected:

		/** remove a clique: warning, can result in a forest */
		void removeClique(sharedClique clique);

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

		/**
		 * Find parent clique of a conditional, given an IndexTable constructed from an ordering.
		 * It will look at all parents and return the one with the lowest index in the ordering.
		 */
		Symbol findParentClique(const std::list<Symbol>& parents, const IndexTable<Symbol>& index) const;

		/** insert a new conditional */
		void insert(const sharedConditional& conditional, const IndexTable<Symbol>& index);

		/** number of cliques */
		inline size_t size() const {
			if(root_)
				return root_->treeSize();
			else
				return 0;
		}

		/** return root clique */
		sharedClique root() const {
			return root_;
		}

		/** find the clique to which key belongs */
		sharedClique operator[](const Symbol& key) const {
			typename Nodes::const_iterator it = nodes_.find(key);
			if (it == nodes_.end()) throw(std::invalid_argument(
					"BayesTree::operator['" + (std::string)key + "']: key not found"));
			sharedClique clique = it->second;
			return clique;
		}

		/** return marginal on any variable */
		template<class Factor>
		FactorGraph<Factor> marginal(const Symbol& key) const;

		/** return marginal on any variable, as a Bayes Net */
		template<class Factor>
		BayesNet<Conditional> marginalBayesNet(const Symbol& key) const;

		/** return joint on two variables */
		template<class Factor>
		FactorGraph<Factor> joint(const Symbol& key1, const Symbol& key2) const;

		/** return joint on two variables as a BayesNet */
		template<class Factor>
		BayesNet<Conditional> jointBayesNet(const Symbol& key1, const Symbol& key2) const;

		/**
		 * Remove all nodes
		 */
		void clear();

		/**
		 * Remove path from clique to root and return that path as factors
		 * plus a list of orphaned subtree roots. Used in removeTop below.
		 */
		void removePath(sharedClique clique, BayesNet<Conditional>& bn, Cliques& orphans);

		/**
		 * Given a list of keys, turn "contaminated" part of the tree back into a factor graph.
		 * Factors and orphans are added to the in/out arguments.
		 */
		void removeTop(const std::list<Symbol>& keys,	BayesNet<Conditional>& bn, Cliques& orphans);

	}; // BayesTree

} /// namespace gtsam
