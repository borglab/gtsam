/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

#include "BayesTree.h"

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::BayesTree() {
	}

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::BayesTree(BayesChain<Conditional>& bayesChain) {
		list<string> ordering;// = bayesChain.ordering();
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::print(const std::string& s) const {
		cout << s << ": size == " << nodes_.size() << endl;
		if (nodes_.empty()) return;
		nodes_[0]->printTree("");
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesTree<Conditional>::equals(const BayesTree<Conditional>& other,
			double tol) const {
		return false;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::insert(string key, conditional_ptr conditional) {

		// get any parent
		list<string> parents = conditional->parents();

		// if no parents, start a new root clique
		if (parents.empty()) {
			node_ptr root(new Node(key, conditional));
			nodes_.push_back(root);
			nodeMap_.insert(make_pair(key, 0));
			return;
		}

		// otherwise, find the parent clique
		string parent = parents.front();
		NodeMap::const_iterator it = nodeMap_.find(parent);
		if (it == nodeMap_.end()) throw(invalid_argument(
				"BayesTree::insert: parent with key " + key + "was not yet inserted"));
		int index = it->second;
		node_ptr parent_clique = nodes_[index];

		// if the parents and parent clique have the same size, add to parent clique
		if (parent_clique->size() == parents.size()) {
			nodeMap_.insert(make_pair(key, index));
			parent_clique->add(key, conditional);
			return;
		}

		// otherwise, start a new clique and add it to the tree
		node_ptr new_clique(new Node(key, conditional));
		new_clique->parent_ = parent_clique;
		parent_clique->children_.push_back(new_clique);
		nodeMap_.insert(make_pair(key, nodes_.size()));
		nodes_.push_back(new_clique);
	}

/* ************************************************************************* */

} /// namespace gtsam
