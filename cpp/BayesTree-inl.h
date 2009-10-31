/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

#include <boost/foreach.hpp>
#include "BayesTree.h"

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	template<class Conditional>
	Front<Conditional>::Front(string key, cond_ptr conditional) {
		add(key, conditional);
		separator_ = conditional->parents();
	}

	/* ************************************************************************* */
	template<class Conditional>
	void Front<Conditional>::print(const string& s) const {
		cout << s;
		BOOST_FOREACH(string key, keys_) cout << " " << key;
		if (!separator_.empty()) {
			cout << " :";
			BOOST_FOREACH(string key, separator_)
			cout << " " << key;
		}
		cout << endl;
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool Front<Conditional>::equals(const Front<Conditional>& other, double tol) const {
		return (keys_ == other.keys_) &&
		equal(conditionals_.begin(),conditionals_.end(),other.conditionals_.begin(),equals_star<Conditional>);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void Front<Conditional>::add(string key, cond_ptr conditional) {
		keys_.push_front(key);
		conditionals_.push_front(conditional);
	}

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::BayesTree() {
	}

	/* ************************************************************************* */
	// TODO: traversal is O(n*log(n)) but could be O(n) with better bayesChain
	template<class Conditional>
	BayesTree<Conditional>::BayesTree(BayesChain<Conditional>& bayesChain, bool verbose) {
		list<string> reverseOrdering = bayesChain.keys();
		BOOST_FOREACH(string key, reverseOrdering)
			insert(key,bayesChain[key],verbose);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::print(const string& s) const {
		cout << s << ": size == " << nodes_.size() << endl;
		if (nodes_.empty()) return;
		nodes_[0]->printTree("");
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesTree<Conditional>::equals(const BayesTree<Conditional>& other,
			double tol) const {
		return size()==other.size() &&
		equal(nodeMap_.begin(),nodeMap_.end(),other.nodeMap_.begin()) &&
		equal(nodes_.begin(),nodes_.end(),other.nodes_.begin(),equals_star<Node>);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::insert(string key, conditional_ptr conditional, bool verbose) {

		if (verbose) cout << "Inserting " << key << "| ";

		// get parents
		list<string> parents = conditional->parents();
		if (verbose) BOOST_FOREACH(string p, parents) cout << p << " ";
		if (verbose) cout << endl;

		// if no parents, start a new root clique
		if (parents.empty()) {
			if (verbose) cout << "Creating root clique" << endl;
			node_ptr root(new Node(key, conditional));
			nodes_.push_back(root);
			nodeMap_.insert(make_pair(key, 0));
			return;
		}

		// otherwise, find the parent clique
		string parent = parents.front();
		NodeMap::const_iterator it = nodeMap_.find(parent);
		if (it == nodeMap_.end()) throw(invalid_argument(
						"BayesTree::insert('"+key+"'): parent '" + parent + "' was not yet inserted"));
		int index = it->second;
		node_ptr parent_clique = nodes_[index];

		// if the parents and parent clique have the same size, add to parent clique
		if (parent_clique->size() == parents.size()) {
			if (verbose) cout << "Adding to clique " << index << endl;
			nodeMap_.insert(make_pair(key, index));
			parent_clique->add(key, conditional);
			return;
		}

		// otherwise, start a new clique and add it to the tree
		if (verbose) cout << "Starting new clique" << endl;
		node_ptr new_clique(new Node(key, conditional));
		new_clique->parent_ = parent_clique;
		parent_clique->children_.push_back(new_clique);
		nodeMap_.insert(make_pair(key, nodes_.size()));
		nodes_.push_back(new_clique);
	}

	/* ************************************************************************* */

} /// namespace gtsam
