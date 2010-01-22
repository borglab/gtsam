/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 */

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include "Conditional.h"
#include "BayesTree.h"
#include "Ordering.h"
#include "inference-inl.h"

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
			BOOST_FOREACH(const sharedConditional& conditional, this->conditionals_)
				cout << " " << (string)(conditional->key());
			if (!separator_.empty()) {
				cout << " :";
				BOOST_FOREACH(const Symbol& key, separator_)
					cout << " " << (std::string)key;
			}
			cout << endl;
		}

	/* ************************************************************************* */
	template<class Conditional>
	size_t BayesTree<Conditional>::Clique::treeSize() const {
		size_t size = 1;
		BOOST_FOREACH(shared_ptr child, children_)
			size += child->treeSize();
		return size;
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
	typename BayesTree<Conditional>::CliqueData
	BayesTree<Conditional>::getCliqueData() const {
		CliqueData data;
		getCliqueData(data, root_);
		return data;
	}

	template<class Conditional>
	void BayesTree<Conditional>::getCliqueData(CliqueData& data,
			BayesTree<Conditional>::sharedClique clique) const {
		data.conditionalSizes.push_back(clique->conditionals_.size());
		data.separatorSizes.push_back(clique->separator_.size());
		BOOST_FOREACH(sharedClique c, clique->children_) {
			getCliqueData(data, c);
		}
	}

	template<class Conditional>
	typename BayesTree<Conditional>::CliqueStats
	BayesTree<Conditional>::CliqueData::getStats() const {
		CliqueStats stats;

		double sum = 0.0;
		size_t max = 0;
		BOOST_FOREACH(size_t s, conditionalSizes) {
			sum += (double)s;
			if(s > max) max = s;
		}
		stats.avgConditionalSize = sum / (double)conditionalSizes.size();
		stats.maxConditionalSize = max;

		sum = 0.0;
		max = 1;
		BOOST_FOREACH(size_t s, separatorSizes) {
			sum += (double)s;
			if(s > max) max = s;
		}
		stats.avgSeparatorSize = sum / (double)separatorSizes.size();
		stats.maxSeparatorSize = max;

		return stats;
	}

	/* ************************************************************************* */
	// The shortcut density is a conditional P(S|R) of the separator of this
	// clique on the root. We can compute it recursively from the parent shortcut
	// P(Sp|R) as \int P(Fp|Sp) P(Sp|R), where Fp are the frontal nodes in p
	// TODO, why do we actually return a shared pointer, why does eliminate?
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	BayesNet<Conditional>
	BayesTree<Conditional>::Clique::shortcut(shared_ptr R) {
		// A first base case is when this clique or its parent is the root,
		// in which case we return an empty Bayes net.

		if (R.get()==this || parent_==R) {
			BayesNet<Conditional> empty;
			return empty;
		}

		// The parent clique has a Conditional for each frontal node in Fp
		// so we can obtain P(Fp|Sp) in factor graph form
		FactorGraph<Factor> p_Fp_Sp(*parent_);

		// If not the base case, obtain the parent shortcut P(Sp|R) as factors
		FactorGraph<Factor> p_Sp_R(parent_->shortcut<Factor>(R));

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
		BOOST_FOREACH(const Symbol& key, R->ordering()) {
			integrands.remove(key);
			ordering.remove(key);
		}

		// remove any variables in the separator, after this integrands = Cp\R\S
		BOOST_FOREACH(const Symbol& key, separator_) integrands.remove(key);

		// form the ordering as [Cp\R\S S\R]
		BOOST_REVERSE_FOREACH(const Symbol& key, integrands) ordering.push_front(key);

		// eliminate to get marginal
		BayesNet<Conditional> p_S_R = eliminate<Factor,Conditional>(p_Cp_R,ordering);

		// remove all integrands
		BOOST_FOREACH(const Symbol& key, integrands) p_S_R.pop_front();

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
	FactorGraph<Factor>
	BayesTree<Conditional>::Clique::marginal(shared_ptr R) {
		// If we are the root, just return this root
		if (R.get()==this) return *R;

		// Combine P(F|S), P(S|R), and P(R)
		BayesNet<Conditional> p_FSR = this->shortcut<Factor>(R);
		p_FSR.push_front(*this);
		p_FSR.push_back(*R);

		// Find marginal on the keys we are interested in
		return marginalize<Factor,Conditional>(p_FSR,keys());
	}

	/* ************************************************************************* */
	// P(C1,C2) = \int_R P(F1|S1) P(S1|R) P(F2|S1) P(S2|R) P(R)
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	FactorGraph<Factor>
	BayesTree<Conditional>::Clique::joint(shared_ptr C2, shared_ptr R) {
		// For now, assume neither is the root

		// Combine P(F1|S1), P(S1|R), P(F2|S2), P(S2|R), and P(R)
		sharedBayesNet bn(new BayesNet<Conditional>);
		if (!isRoot())     bn->push_back(*this);                   // P(F1|S1)
		if (!isRoot())     bn->push_back(shortcut<Factor>(R));     // P(S1|R)
		if (!C2->isRoot()) bn->push_back(*C2);                     // P(F2|S2)
		if (!C2->isRoot()) bn->push_back(C2->shortcut<Factor>(R)); // P(S2|R)
		bn->push_back(*R);                                         // P(R)

		// Find the keys of both C1 and C2
		Ordering keys12 = keys();
		BOOST_FOREACH(const Symbol& key,C2->keys()) keys12.push_back(key);
		keys12.unique();

		// Calculate the marginal
		return marginalize<Factor,Conditional>(*bn,keys12);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::Cliques::print(const std::string& s) const {
		cout << s << ":\n";
		BOOST_FOREACH(sharedClique clique, *this)
				clique->printTree();
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesTree<Conditional>::Cliques::equals(const Cliques& other, double tol) const {
		return other == *this;
	}

	/* ************************************************************************* */
	template<class Conditional>
	typename BayesTree<Conditional>::sharedClique BayesTree<Conditional>::addClique
	(const sharedConditional& conditional, sharedClique parent_clique) {
		sharedClique new_clique(new Clique(conditional));
		nodes_.insert(make_pair(conditional->key(), new_clique));
		if (parent_clique != NULL) {
			new_clique->parent_ = parent_clique;
			parent_clique->children_.push_back(new_clique);
		}
		return new_clique;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::removeClique(sharedClique clique) {

		if (clique->isRoot())
			root_.reset();
		else // detach clique from parent
	    clique->parent_->children_.remove(clique);

	  // orphan my children
		BOOST_FOREACH(sharedClique child, clique->children_)
	  	child->parent_.reset();

	  BOOST_FOREACH(const Symbol& key, clique->ordering()) {
			nodes_.erase(key);
	  }
	}

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::BayesTree() {
	}

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::BayesTree(const BayesNet<Conditional>& bayesNet) {
		IndexTable<Symbol> index(bayesNet.ordering());
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			insert(*rit, index);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::print(const string& s) const {
		if (root_.use_count() == 0) {
			printf("WARNING: BayesTree.print encountered a forest...\n");
			return;
		}
		cout << s << ": size == " << size() << endl;
		if (nodes_.empty()) return;
		root_->printTree("");
	}

	/* ************************************************************************* */
	// binary predicate to test equality of a pair for use in equals
	template<class Conditional>
	bool check_pair(
			const pair<Symbol,typename BayesTree<Conditional>::sharedClique >& v1,
			const pair<Symbol,typename BayesTree<Conditional>::sharedClique >& v2
	) {
		return v1.first == v2.first && v1.second->equals(*(v2.second));
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesTree<Conditional>::equals(const BayesTree<Conditional>& other,
			double tol) const {
		return size()==other.size() &&
				equal(nodes_.begin(),nodes_.end(),other.nodes_.begin(),check_pair<Conditional>);
	}

	/* ************************************************************************* */
	template<class Conditional>
	Symbol BayesTree<Conditional>::findParentClique(const list<Symbol>& parents,
			const IndexTable<Symbol>& index) const {
		boost::optional<Symbol> parentCliqueRepresentative;
		boost::optional<size_t> lowest;
		BOOST_FOREACH(const Symbol& p, parents) {
			size_t i = index(p);
			if (!lowest || i<*lowest) {
				lowest.reset(i);
				parentCliqueRepresentative.reset(p);
			}
		}
		if (!lowest) throw
			invalid_argument("BayesTree::findParentClique: no parents given or key not present in index");
		return *parentCliqueRepresentative;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::insert(const sharedConditional& conditional,
			const IndexTable<Symbol>& index)
	{
		// get key and parents
		const Symbol& key = conditional->key();
		list<Symbol> parents = conditional->parents(); // todo: const reference?

		// if no parents, start a new root clique
		if (parents.empty()) {
			root_ = addClique(conditional);
			return;
		}

		// otherwise, find the parent clique by using the index data structure
		// to find the lowest-ordered parent
		Symbol parentRepresentative = findParentClique(parents, index);
		sharedClique parent_clique = (*this)[parentRepresentative];

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
	FactorGraph<Factor>
	BayesTree<Conditional>::marginal(const Symbol& key) const {

		// get clique containing key
		sharedClique clique = (*this)[key];

		// calculate or retrieve its marginal
		FactorGraph<Factor> cliqueMarginal = clique->marginal<Factor>(root_);

		// create an ordering where only the requested key is not eliminated
		Ordering ord = clique->keys();
		ord.remove(key);

		// partially eliminate, remaining factor graph is requested marginal
		eliminate<Factor,Conditional>(cliqueMarginal,ord);
		return cliqueMarginal;
	}

	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	BayesNet<Conditional>
	BayesTree<Conditional>::marginalBayesNet(const Symbol& key) const {

		// calculate marginal as a factor graph
	  FactorGraph<Factor> fg = this->marginal<Factor>(key);

		// eliminate further to Bayes net
		return eliminate<Factor,Conditional>(fg,Ordering(key));
	}

	/* ************************************************************************* */
	// Find two cliques, their joint, then marginalizes
	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	FactorGraph<Factor>
	BayesTree<Conditional>::joint(const Symbol& key1, const Symbol& key2) const {

		// get clique C1 and C2
		sharedClique C1 = (*this)[key1], C2 = (*this)[key2];

		// calculate joint
		FactorGraph<Factor> p_C1C2 = C1->joint<Factor>(C2,root_);

		// create an ordering where both requested keys are not eliminated
		Ordering ord = p_C1C2.keys();
		ord.remove(key1);
		ord.remove(key2);

		// partially eliminate, remaining factor graph is requested joint
		// TODO, make eliminate functional
		eliminate<Factor,Conditional>(p_C1C2,ord);
		return p_C1C2;
	}

	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	BayesNet<Conditional>
	BayesTree<Conditional>::jointBayesNet(const Symbol& key1, const Symbol& key2) const {

		// calculate marginal as a factor graph
	  FactorGraph<Factor> fg = this->joint<Factor>(key1,key2);

		// eliminate further to Bayes net
		Ordering ordering;
		ordering += key1, key2;
		return eliminate<Factor,Conditional>(fg,ordering);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::clear() {
		// Remove all nodes and clear the root pointer
		nodes_.clear();
		root_.reset();
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::removePath(sharedClique clique,
			BayesNet<Conditional>& bn, typename BayesTree<Conditional>::Cliques& orphans) {

		// base case is NULL, if so we do nothing and return empties above
		if (clique!=NULL) {

			// remove the clique from orphans in case it has been added earlier
			orphans.remove(clique);

			// remove me
			this->removeClique(clique);

			// remove path above me
			this->removePath(clique->parent_, bn, orphans);

			// add children to list of orphans (splice also removed them from clique->children_)
			orphans.splice (orphans.begin(), clique->children_);

			bn.push_back(*clique);

		}
	}

	/* ************************************************************************* */
	template<class Conditional>
  void BayesTree<Conditional>::removeTop(const list<Symbol>& keys,
  		BayesNet<Conditional>& bn, typename BayesTree<Conditional>::Cliques& orphans) {

		// process each key of the new factor
		BOOST_FOREACH(const Symbol& key, keys)
			try {
				// get the clique
				sharedClique clique = (*this)[key];

				// remove path from clique to root
				this->removePath(clique, bn, orphans);

			} catch (std::invalid_argument e) {
			}
	}

/* ************************************************************************* */

}
/// namespace gtsam
