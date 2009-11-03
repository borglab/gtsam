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
	BayesTree<Conditional>::Node::Node(const boost::shared_ptr<Conditional>& conditional) {
			separator_ = conditional->parents();
			this->push_back(conditional);
		}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::Node::print(const string& s) const {
			cout << s;
			BOOST_REVERSE_FOREACH(const conditional_ptr& conditional, this->conditionals_)
				cout << " " << conditional->key();
			if (!separator_.empty()) {
				cout << " :";
				BOOST_FOREACH(string key, separator_)
				cout << " " << key;
			}
			cout << endl;
		}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::Node::printTree(const string& indent) const {
			print(indent);
			BOOST_FOREACH(shared_ptr child, children_)
				child->printTree(indent+"  ");
		}

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::BayesTree() {
	}

	/* ************************************************************************* */
	// TODO: traversal is O(n*log(n)) but could be O(n) with better bayesNet
	template<class Conditional>
	BayesTree<Conditional>::BayesTree(const BayesNet<Conditional>& bayesNet, bool verbose) {
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			insert(*rit,verbose);
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
		equal(nodes_.begin(),nodes_.end(),other.nodes_.begin(),equals_star<Node>(tol));
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::insert(const boost::shared_ptr<Conditional>& conditional, bool verbose) {

		string key =  conditional->key();
		if (verbose) cout << "Inserting " << key << "| ";

		// get parents
		list<string> parents = conditional->parents();
		if (verbose) BOOST_FOREACH(string p, parents) cout << p << " ";
		if (verbose) cout << endl;

		// if no parents, start a new root clique
		if (parents.empty()) {
			if (verbose) cout << "Creating root clique" << endl;
			node_ptr root(new Node(conditional));
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
		if (verbose) cout << "Parent clique " << index << " of size " << parent_clique->size() << endl;

		// if the parents and parent clique have the same size, add to parent clique
		if (parent_clique->size() == parents.size()) {
			if (verbose) cout << "Adding to clique " << index << endl;
			nodeMap_.insert(make_pair(key, index));
			parent_clique->push_front(conditional);
			return;
		}

		// otherwise, start a new clique and add it to the tree
		if (verbose) cout << "Starting new clique" << endl;
		node_ptr new_clique(new Node(conditional));
		new_clique->parent_ = parent_clique;
		parent_clique->children_.push_back(new_clique);
		nodeMap_.insert(make_pair(key, nodes_.size()));
		nodes_.push_back(new_clique);
	}

	/* ************************************************************************* */

}
/// namespace gtsam
