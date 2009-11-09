/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

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
		return marginals<Factor>(*p_FSR,keys());
	}

	/* ************************************************************************* */
	// P(C1,C2) = \int_R P(F1|S1) P(S1|R) P(F2|S1) P(S2|R) P(R)
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	BayesNet<Conditional>
	BayesTree<Conditional>::Clique::joint(shared_ptr C2, shared_ptr R) {
		// For now, assume neither is the root

		// Combine P(F1|S1), P(S1|R), P(F2|S2), P(S2|R), and P(R)
		sharedBayesNet p_FSR = this->shortcut<Factor>(R);
		p_FSR->push_front(*this);
		p_FSR->push_front(*C2->shortcut<Factor>(R));
		p_FSR->push_front(*C2);
		p_FSR->push_back(*R);

		// Find the keys of both C1 and C2
		Ordering keys12 = keys();
		BOOST_FOREACH(string key,C2->keys()) keys12.push_back(key);
		keys12.unique();

		// Calculate the marginal
		return marginals<Factor>(*p_FSR,keys12);
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
		return marginals<Factor>(cliqueMarginal,Ordering(key));
	}

	/* ************************************************************************* */
	// Find two cliques, their joint, then marginalizes
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	BayesNet<Conditional>
	BayesTree<Conditional>::joint(const std::string& key1, const std::string& key2) const {

		// get clique C1 and C2
		sharedClique C1 = (*this)[key1], C2 = (*this)[key2];

		// calculate joint
		BayesNet<Conditional> p_C1C2 = C1->joint<Factor>(C2,root_);

		// Get the marginal on the two keys
		Ordering ordering;
		ordering += key1, key2;
		return marginals<Factor>(p_C1C2,ordering);
	}

	/* ************************************************************************* */

}
/// namespace gtsam
