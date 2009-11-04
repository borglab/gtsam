/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

#include <boost/foreach.hpp>
#include "BayesTree.h"
#include "FactorGraph.h"

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
	BayesTree<Conditional>::BayesTree(const BayesNet<Conditional>& bayesNet) {
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			insert(*rit);
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
	void BayesTree<Conditional>::addClique
	(const boost::shared_ptr<Conditional>& conditional, node_ptr parent_clique)
	{
		node_ptr new_clique(new Node(conditional));
		nodeMap_.insert(make_pair(conditional->key(), nodes_.size()));
		nodes_.push_back(new_clique);
		if (parent_clique==NULL) return;
		new_clique->parent_ = parent_clique;
		parent_clique->children_.push_back(new_clique);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::insert
	(const boost::shared_ptr<Conditional>& conditional)
	{
		// get key and parents
		string key = conditional->key();
		list<string> parents = conditional->parents();

		// if no parents, start a new root clique
		if (parents.empty()) {
			addClique(conditional);
			return;
		}

		// otherwise, find the parent clique
		string parent = parents.front();
		NodeMap::const_iterator it = nodeMap_.find(parent);
		if (it == nodeMap_.end()) throw(invalid_argument(
				"BayesTree::insert('"+key+"'): parent '" + parent + "' not yet inserted"));
		int parent_index = it->second;
		node_ptr parent_clique = nodes_[parent_index];

		// if the parents and parent clique have the same size, add to parent clique
		if (parent_clique->size() == parents.size()) {
			nodeMap_.insert(make_pair(key, parent_index));
			parent_clique->push_front(conditional);
			return;
		}

		// otherwise, start a new clique and add it to the tree
		addClique(conditional,parent_clique);
	}

	/* ************************************************************************* */
	template<class Conditional>
	boost::shared_ptr<Conditional> BayesTree<Conditional>::marginal(const string& key) const {

		// find the clique to which key belongs
		NodeMap::const_iterator it = nodeMap_.find(key);
		if (it == nodeMap_.end()) throw(invalid_argument(
						"BayesTree::marginal('"+key+"'): key not found"));

		// find all cliques on the path to the root
		// FactorGraph

		boost::shared_ptr<Conditional> result(new Conditional);
		return result;
	}

	/* ************************************************************************* */

}
/// namespace gtsam
