/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

#include <boost/foreach.hpp>
#include "BayesTree.h"
#include "FactorGraph-inl.h"

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
		root_->printTree("");
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesTree<Conditional>::equals(const BayesTree<Conditional>& other,
			double tol) const {
		return size()==other.size();
		//&& equal(nodes_.begin(),nodes_.end(),other.nodes_.begin(),equals_star<Node>(tol));
	}

	/* ************************************************************************* */
	template<class Conditional>
	boost::shared_ptr<typename BayesTree<Conditional>::Node> BayesTree<Conditional>::addClique
	(const boost::shared_ptr<Conditional>& conditional, node_ptr parent_clique)
	{
		node_ptr new_clique(new Node(conditional));
		nodes_.insert(make_pair(conditional->key(), new_clique));
		if (parent_clique!=NULL) {
			new_clique->parent_ = parent_clique;
			parent_clique->children_.push_back(new_clique);
		}
		return new_clique;
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
			root_ = addClique(conditional);
			return;
		}

		// otherwise, find the parent clique
		string parent = parents.front();
		node_ptr parent_clique = (*this)[parent];

		// if the parents and parent clique have the same size, add to parent clique
		if (parent_clique->size() == parents.size()) {
			nodes_.insert(make_pair(key, parent_clique));
			parent_clique->push_front(conditional);
			return;
		}

		// otherwise, start a new clique and add it to the tree
		addClique(conditional,parent_clique);
	}

	/* ************************************************************************* */
	// Desired: recursive, memoizing version
	// Once we know the clique, can we do all with Nodes ?
	// Sure, as P(x) = \int P(C|root)
	// The natural cache is P(C|root), memoized, of course, in the clique C
	// When any marginal is asked for, we calculate P(C|root) = P(C|Pi)P(Pi|root)
	// Super-naturally recursive !!!!!
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	boost::shared_ptr<Conditional> BayesTree<Conditional>::marginal(const string& key) const {

		// get clique containing key, and remove all factors below key
		node_ptr clique = (*this)[key];
		Ordering ordering = clique->ordering();
		FactorGraph<Factor> graph(*clique);
		while(ordering.front()!=key) {
			graph.findAndRemoveFactors(ordering.front());
			ordering.pop_front();
		}

		// find all cliques on the path to the root and turn into factor graph
		while (clique->parent_!=NULL) {
			// move up the tree
			clique = clique->parent_;

			// extend ordering
			Ordering cliqueOrdering = clique->ordering();
			ordering.splice (ordering.end(), cliqueOrdering);

			// extend factor graph
			FactorGraph<Factor> cliqueGraph(*clique);
			typename FactorGraph<Factor>::const_iterator factor=cliqueGraph.begin();
			for(; factor!=cliqueGraph.end(); factor++)
				graph.push_back(*factor);
		}

		// TODO: can we prove reverse ordering is efficient?
		ordering.reverse();

		// eliminate to get marginal
		boost::shared_ptr<BayesNet<Conditional> > bayesNet;
		typename boost::shared_ptr<BayesNet<Conditional> > chordalBayesNet =
				graph.eliminate(bayesNet,ordering);

		return chordalBayesNet->back(); // the root is the marginal
	}

	/* ************************************************************************* */

}
/// namespace gtsam
