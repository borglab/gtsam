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
	BayesTree<Conditional>::Clique::Clique(const sharedConditional& conditional) {
			separator_ = conditional->parents();
			this->push_back(conditional);
		}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::Clique::print(const string& s) const {
			cout << s;
			BOOST_REVERSE_FOREACH(const sharedConditional& conditional, this->conditionals_)
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
	void BayesTree<Conditional>::Clique::printTree(const string& indent) const {
			print(indent);
			BOOST_FOREACH(shared_ptr child, children_)
				child->printTree(indent+"  ");
		}

	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	typename BayesTree<Conditional>::sharedBayesNet
	BayesTree<Conditional>::Clique::shortcut(shared_ptr R) {
		// The shortcut density is a conditional P(S|R) of the separator of this
		// clique on the root. We can compute it recursively from the parent shortcut
		// P(Sp|R) as \int P(Fp|Sp) P(Sp|R), where Fp are the frontal nodes in p

		// A first base case is when this clique or its parent is the root,
		// in which case we return an empty Bayes net.
		if (R.get()==this || parent_==R) {
			sharedBayesNet empty(new BayesNet<Conditional>);
			return empty;
		}

		// The parent clique has a Conditional for each frontal node in Fp
		// so we can obtain P(Fp|Sp) in factor graph form
		FactorGraph<Factor> p_Fp_Sp(*parent_);
		//p_Fp_Sp.print("p_Fp_Sp");

		// If not the base case, obtain the parent shortcut P(Sp|R) as factors
		FactorGraph<Factor> p_Sp_R(*parent_->shortcut<Factor>(R));
		//p_Sp_R.print("p_Sp_R");

		// now combine P(Cp|R) = P(Fp|Sp) * P(Sp|R)
		FactorGraph<Factor> p_Cp_R = combine(p_Fp_Sp, p_Sp_R);

		// Eliminate into a Bayes net with ordering designed to integrate out
		// any variables not in *our* separator. Variables to integrate out must be
		// eliminated first hence the desired ordering is [Cp\S S].
		// However, an added wrinkle is that Cp might overlap with the root.
		// Keys corresponding to the root should not be added to the ordering at all.

		// Get the key list Cp=Fp+Sp, which will form the basis for the integrands
		Ordering integrands;
		{
		Ordering Fp = parent_->ordering(), Sp = parent_->separator_;
		integrands.splice(integrands.end(),Fp);
		integrands.splice(integrands.end(),Sp);
		}

		// Start ordering with the separator
		Ordering ordering = separator_;

		// remove any variables in the root, after this integrands = Cp\R, ordering = S\R
		BOOST_FOREACH(string key, R->ordering()) {
			integrands.remove(key);
			ordering.remove(key);
		}

		// remove any variables in the separator, after this integrands = Cp\R\S
		BOOST_FOREACH(string key, separator_) integrands.remove(key);

		// form the ordering as [Cp\R\S S\R]
		BOOST_REVERSE_FOREACH(string key, integrands) ordering.push_front(key);

		// eliminate to get marginal
		sharedBayesNet p_S_R = _eliminate<Factor,Conditional>(p_Cp_R,ordering);

		// remove all integrands
		BOOST_FOREACH(string key, integrands) p_S_R->pop_front();

		// return the parent shortcut P(Sp|R)
		return p_S_R;
	}

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::BayesTree() {
	}

	/* ************************************************************************* */
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
		//&& equal(nodes_.begin(),nodes_.end(),other.nodes_.begin(),equals_star<Clique>(tol));
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::insert(const sharedConditional& conditional)
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
		sharedClique parent_clique = (*this)[parent];

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
	typename BayesTree<Conditional>::sharedConditional
	BayesTree<Conditional>::marginal(const string& key) const {

		// get clique containing key, and remove all factors below key
		sharedClique clique = (*this)[key];
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
		sharedBayesNet chordalBayesNet = _eliminate<Factor,Conditional>(graph,ordering);

		return chordalBayesNet->back(); // the root is the marginal
	}

	/* ************************************************************************* */

}
/// namespace gtsam
