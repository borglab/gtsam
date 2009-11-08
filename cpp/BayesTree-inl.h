/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

#include <boost/foreach.hpp>
#include "BayesTree.h"
#include "FactorGraph-inl.h"
#include "BayesNet-inl.h"

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
	Ordering BayesTree<Conditional>::Clique::keys() const {
		Ordering frontal_keys = this->ordering(), keys = separator_;
		keys.splice(keys.begin(),frontal_keys);
		return keys;
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
	// The shortcut density is a conditional P(S|R) of the separator of this
	// clique on the root. We can compute it recursively from the parent shortcut
	// P(Sp|R) as \int P(Fp|Sp) P(Sp|R), where Fp are the frontal nodes in p
	// TODO, why do we actually return a shared pointer, why does eliminate?
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	typename BayesTree<Conditional>::sharedBayesNet
	BayesTree<Conditional>::Clique::shortcut(shared_ptr R) {
		// A first base case is when this clique or its parent is the root,
		// in which case we return an empty Bayes net.
		if (R.get()==this || parent_==R)
			return sharedBayesNet(new BayesNet<Conditional>);

		// The parent clique has a Conditional for each frontal node in Fp
		// so we can obtain P(Fp|Sp) in factor graph form
		FactorGraph<Factor> p_Fp_Sp(*parent_);

		// If not the base case, obtain the parent shortcut P(Sp|R) as factors
		FactorGraph<Factor> p_Sp_R(*parent_->shortcut<Factor>(R));

		// now combine P(Cp|R) = P(Fp|Sp) * P(Sp|R)
		FactorGraph<Factor> p_Cp_R = combine(p_Fp_Sp, p_Sp_R);

		// Eliminate into a Bayes net with ordering designed to integrate out
		// any variables not in *our* separator. Variables to integrate out must be
		// eliminated first hence the desired ordering is [Cp\S S].
		// However, an added wrinkle is that Cp might overlap with the root.
		// Keys corresponding to the root should not be added to the ordering at all.

		// Get the key list Cp=Fp+Sp, which will form the basis for the integrands
		Ordering integrands = parent_->keys();

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
	// P(C) = \int_R P(F|S) P(S|R) P(R)
	// TODO: Maybe we should integrate given parent marginal P(Cp),
	// \int(Cp\S) P(F|S)P(S|Cp)P(Cp)
	// Because the root clique could be very big.
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	BayesNet<Conditional>
	BayesTree<Conditional>::Clique::marginal(shared_ptr R) {
		// If we are the root, just return this root
		if (R.get()==this) return *R;

		// Combine P(F|S), P(S|R), and P(R)
		sharedBayesNet p_FSR = this->shortcut<Factor>(R);
		p_FSR->push_front(*this);
		p_FSR->push_back(*R);

		// Find marginal on the keys we are interested in
		BayesNet<Conditional> marginal = marginals<Factor>(*p_FSR,keys());
		return marginal;
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
	// First finds clique marginal then marginalizes that
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	BayesNet<Conditional>
	BayesTree<Conditional>::marginal(const string& key) const {

		// get clique containing key
		sharedClique clique = (*this)[key];

		// calculate or retrieve its marginal
		BayesNet<Conditional> cliqueMarginal = clique->marginal<Factor>(root_);

		// Get the marginal on the single key
		BayesNet<Conditional> marginal = marginals<Factor>(cliqueMarginal,Ordering(key));

		return marginal;
	}

	/* ************************************************************************* */

}
/// namespace gtsam
