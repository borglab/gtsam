/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 */

#pragma once

#include <iostream>
#include <algorithm>

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/format.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <fstream>
using namespace boost::assign;
namespace lam = boost::lambda;

#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/inference-inl.h>

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::Clique::Clique() {}

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::Clique::Clique(const sharedConditional& conditional) {
		separator_.assign(conditional->parents().begin(), conditional->parents().end());
		this->push_back(conditional);
	}

	/* ************************************************************************* */
	template<class Conditional>
	BayesTree<Conditional>::Clique::Clique(const BayesNet<Conditional>& bayesNet)
	: BayesNet<Conditional>(bayesNet) {
	  if(bayesNet.size() > 0) {
#ifndef NDEBUG
	    // Debug check that each parent variable is either a frontal variable in
	    // a later conditional in the Bayes net fragment, or that the parent is
	    // also a parent of the last conditional.  This checks that the fragment
	    // is "enough like a clique".  todo: this should really check that the
	    // fragment *is* a clique.
	    if(bayesNet.size() > 1) {
	      typename BayesNet<Conditional>::const_reverse_iterator cond = bayesNet.rbegin();
	      ++ cond;
	      typename Conditional::shared_ptr lastConditional = *cond;
	      for( ; cond != bayesNet.rend(); ++cond)
	        for(typename Conditional::const_iterator parent=(*cond)->beginParents(); parent!=(*cond)->endParents(); ++parent) {
	          bool infragment = false;
	          typename BayesNet<Conditional>::const_reverse_iterator parentCond = cond;
	          do {
	            if(*parent == (*parentCond)->key())
	              infragment = true;
	            --parentCond;
	          } while(parentCond != bayesNet.rbegin());
	          assert(infragment || find(lastConditional->beginParents(), lastConditional->endParents(), *parent) != lastConditional->endParents());
	        }
	    }
#endif

//	    separator_.assign((*bayesNet.rbegin())->parents().begin(), (*bayesNet.rbegin())->parents().end());
	    separator_.assign((*bayesNet.rbegin())->beginParents(), (*bayesNet.rbegin())->endParents());
	  }
	}

	/* ************************************************************************* */
	template<class Conditional>
	vector<Index> BayesTree<Conditional>::Clique::keys() const {
	  vector<Index> keys;
	  keys.reserve(this->size() + separator_.size());
	  BOOST_FOREACH(const sharedConditional conditional, *this) {
	    keys.push_back(conditional->key());
	  }
	  keys.insert(keys.end(), separator_.begin(), separator_.end());
		return keys;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::Clique::print(const string& s) const {
			cout << s << "Clique ";
			BOOST_FOREACH(const sharedConditional& conditional, this->conditionals_) { cout << conditional->key() << " "; }
			cout << "| ";
			BOOST_FOREACH(const Index sep, separator_) { cout << sep << " "; }
			cout << "\n";
			BOOST_FOREACH(const sharedConditional& conditional, this->conditionals_) {
				conditional->print("  " + s + "conditional");
//				cout << " " << conditional->key();
			}
//			if (!separator_.empty()) {
//				cout << " :";
//				BOOST_FOREACH(Index key, separator_)
//					cout << " " << key;
//			}
//			cout << endl;
		}

	/* ************************************************************************* */
	template<class Conditional>
	size_t BayesTree<Conditional>::Clique::treeSize() const {
		size_t size = 1;
		BOOST_FOREACH(const shared_ptr& child, children_)
			size += child->treeSize();
		return size;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::Clique::printTree(const string& indent) const {
		print(indent);
		BOOST_FOREACH(const shared_ptr& child, children_)
			child->printTree(indent+"  ");
	}

  /* ************************************************************************* */
  template<class Conditional>
  void BayesTree<Conditional>::Clique::permuteWithInverse(const Permutation& inversePermutation) {
    BayesNet<Conditional>::permuteWithInverse(inversePermutation);
    BOOST_FOREACH(Index& separatorKey, separator_) { separatorKey = inversePermutation[separatorKey]; }
    if(cachedFactor_) cachedFactor_->permuteWithInverse(inversePermutation);
    BOOST_FOREACH(const shared_ptr& child, children_) {
      child->permuteWithInverse(inversePermutation);
    }
  }

  /* ************************************************************************* */
  template<class Conditional>
  bool BayesTree<Conditional>::Clique::permuteSeparatorWithInverse(const Permutation& inversePermutation) {
    bool changed = BayesNet<Conditional>::permuteSeparatorWithInverse(inversePermutation);
#ifndef NDEBUG
    if(!changed) {
      BOOST_FOREACH(Index& separatorKey, separator_) { assert(separatorKey == inversePermutation[separatorKey]); }
      BOOST_FOREACH(const shared_ptr& child, children_) {
        assert(child->permuteSeparatorWithInverse(inversePermutation) == false);
      }
    }
#endif
    if(changed) {
      BOOST_FOREACH(Index& separatorKey, separator_) { separatorKey = inversePermutation[separatorKey]; }
      if(cachedFactor_) cachedFactor_->permuteWithInverse(inversePermutation);
      BOOST_FOREACH(const shared_ptr& child, children_) {
        (void)child->permuteSeparatorWithInverse(inversePermutation);
      }
    }
    return changed;
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

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::saveGraph(const std::string &s) const {
		if (!root_.get()) throw invalid_argument("the root of bayes tree has not been initialized!");
		ofstream of(s.c_str());
		of<< "digraph G{\n";
		saveGraph(of, root_);
		of<<"}";
		of.close();
	}

	template<class Conditional>
	void BayesTree<Conditional>::saveGraph(ostream &s,
			BayesTree<Conditional>::sharedClique clique,
			int parentnum) const {
		static int num = 0;
		bool first = true;
		std::stringstream out;
		out << num;
		string parent = out.str();
		parent += "[label=\"";

		BOOST_FOREACH(boost::shared_ptr<Conditional> c, clique->conditionals_) {
			if(!first) parent += ","; first = false;
			parent += (string(c->key())).c_str();
		}

		if( clique != root_){
			parent += " : ";
			s << parentnum << "->" << num << "\n";
		}

		first = true;
		BOOST_FOREACH(Index sep, clique->separator_) {
			if(!first) parent += ","; first = false;
			parent += (boost::format("%1%")%sep).str();
		}
		parent += "\"];\n";
		s << parent;
		parentnum = num;

		BOOST_FOREACH(sharedClique c, clique->children_) {
			num++;
			saveGraph(s, c, parentnum);
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
	template<class FactorGraph>
	BayesNet<Conditional>
	BayesTree<Conditional>::Clique::shortcut(shared_ptr R) {
		// A first base case is when this clique or its parent is the root,
		// in which case we return an empty Bayes net.

	  sharedClique parent(parent_.lock());

		if (R.get()==this || parent==R) {
			BayesNet<Conditional> empty;
			return empty;
		}

		// The parent clique has a Conditional for each frontal node in Fp
		// so we can obtain P(Fp|Sp) in factor graph form
		FactorGraph p_Fp_Sp(*parent);

		// If not the base case, obtain the parent shortcut P(Sp|R) as factors
		FactorGraph p_Sp_R(parent->shortcut<FactorGraph>(R));

		// now combine P(Cp|R) = P(Fp|Sp) * P(Sp|R)
		FactorGraph p_Cp_R = combine(p_Fp_Sp, p_Sp_R);

		// Eliminate into a Bayes net with ordering designed to integrate out
		// any variables not in *our* separator. Variables to integrate out must be
		// eliminated first hence the desired ordering is [Cp\S S].
		// However, an added wrinkle is that Cp might overlap with the root.
		// Keys corresponding to the root should not be added to the ordering at all.

		typedef set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > FastJSet;

		// Get the key list Cp=Fp+Sp, which will form the basis for the integrands
		vector<Index> parentKeys(parent->keys());
		FastJSet integrands(parentKeys.begin(), parentKeys.end());

		// Start ordering with the separator
		FastJSet separator(separator_.begin(), separator_.end());

		// remove any variables in the root, after this integrands = Cp\R, ordering = S\R
		BOOST_FOREACH(Index key, R->ordering()) {
			integrands.erase(key);
			separator.erase(key);
		}

		// remove any variables in the separator, after this integrands = Cp\R\S
		BOOST_FOREACH(Index key, separator_) integrands.erase(key);

		// form the ordering as [Cp\R\S S\R]
		vector<Index> ordering; ordering.reserve(integrands.size() + separator.size());
		BOOST_FOREACH(Index key, integrands) ordering.push_back(key);
		BOOST_FOREACH(Index key, separator) ordering.push_back(key);

		// eliminate to get marginal
		typename FactorGraph::variableindex_type varIndex(p_Cp_R);
		Permutation toFront = Permutation::PullToFront(ordering, varIndex.size());
		Permutation::shared_ptr toFrontInverse(toFront.inverse());
		BOOST_FOREACH(const typename FactorGraph::sharedFactor& factor, p_Cp_R) {
		  factor->permuteWithInverse(*toFrontInverse);
		}
		varIndex.permute(toFront);
		BayesNet<Conditional> p_S_R = *Inference::EliminateUntil(p_Cp_R, ordering.size(), varIndex);

		// remove all integrands
		for(Index j=0; j<integrands.size(); ++j)
		  p_S_R.pop_front();

		// Undo the permutation on the shortcut
		p_S_R.permuteWithInverse(toFront);

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
	template<class FactorGraph>
	FactorGraph
	BayesTree<Conditional>::Clique::marginal(shared_ptr R) {
		// If we are the root, just return this root
		if (R.get()==this) return *R;

		// Combine P(F|S), P(S|R), and P(R)
		BayesNet<Conditional> p_FSR = this->shortcut<FactorGraph>(R);
		p_FSR.push_front(*this);
		p_FSR.push_back(*R);

		// Find marginal on the keys we are interested in
		return FactorGraph(*Inference::Marginal(FactorGraph(p_FSR), keys()));
	}

//	/* ************************************************************************* */
//	// P(C1,C2) = \int_R P(F1|S1) P(S1|R) P(F2|S1) P(S2|R) P(R)
//	/* ************************************************************************* */
//	template<class Conditional>
//	template<class Factor>
//	pair<FactorGraph<Factor>, Ordering>
//	BayesTree<Conditional>::Clique::joint(shared_ptr C2, shared_ptr R) {
//		// For now, assume neither is the root
//
//		// Combine P(F1|S1), P(S1|R), P(F2|S2), P(S2|R), and P(R)
//		sharedBayesNet bn(new BayesNet<Conditional>);
//		if (!isRoot())     bn->push_back(*this);                   // P(F1|S1)
//		if (!isRoot())     bn->push_back(shortcut<Factor>(R));     // P(S1|R)
//		if (!C2->isRoot()) bn->push_back(*C2);                     // P(F2|S2)
//		if (!C2->isRoot()) bn->push_back(C2->shortcut<Factor>(R)); // P(S2|R)
//		bn->push_back(*R);                                         // P(R)
//
//		// Find the keys of both C1 and C2
//		Ordering keys12 = keys();
//		BOOST_FOREACH(Index key,C2->keys()) keys12.push_back(key);
//		keys12.unique();
//
//		// Calculate the marginal
//		return make_pair(marginalize<Factor,Conditional>(*bn,keys12), keys12);
//	}

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
	typename BayesTree<Conditional>::sharedClique BayesTree<Conditional>::addClique(
	    const sharedConditional& conditional, sharedClique parent_clique) {
		sharedClique new_clique(new Clique(conditional));
		Index key = conditional->key();
		nodes_.resize(std::max(key+1, nodes_.size()));
		nodes_[key] = new_clique;
		if (parent_clique != NULL) {
			new_clique->parent_ = parent_clique;
			parent_clique->children_.push_back(new_clique);
		}
		return new_clique;
	}

	/* ************************************************************************* */
	template<class Conditional>
	typename BayesTree<Conditional>::sharedClique BayesTree<Conditional>::addClique(
	    const sharedConditional& conditional, list<sharedClique>& child_cliques) {
		sharedClique new_clique(new Clique(conditional));
    Index key = conditional->key();
    nodes_.resize(max(key+1, nodes_.size()));
    nodes_[key] = new_clique;
		new_clique->children_ = child_cliques;
		BOOST_FOREACH(sharedClique& child, child_cliques)
			child->parent_ = new_clique;
		return new_clique;
	}

  /* ************************************************************************* */
	template<class Conditional>
	inline void BayesTree<Conditional>::addToCliqueFront(const sharedConditional& conditional, const sharedClique& clique) {
	  static const bool debug = false;
#ifndef NDEBUG
	  // Debug check to make sure the conditional variable is ordered lower than
	  // its parents and that all of its parents are present either in this
	  // clique or its separator.
	  BOOST_FOREACH(Index parent, conditional->parents()) {
	    assert(parent > conditional->key());
	    bool hasParent = false;
	    const Clique& cliquer(*clique);
	    BOOST_FOREACH(const sharedConditional& child, cliquer) {
	      if(child->key() == parent) {
	        hasParent = true;
	        break;
	      }
	    }
	    if(!hasParent)
	      if(find(clique->separator_.begin(), clique->separator_.end(), parent) != clique->separator_.end())
	        hasParent = true;
	    assert(hasParent);
	  }
#endif
	  if(debug) conditional->print("Adding conditional ");
	  if(debug) clique->print("To clique ");
	  Index key = conditional->key();
	  nodes_.resize(std::max(key+1, nodes_.size()));
	  nodes_[key] = clique;
	  clique->push_front(conditional);
	  if(debug) clique->print("Expanded clique is ");
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::removeClique(sharedClique clique) {

		if (clique->isRoot())
			root_.reset();
		else // detach clique from parent
	    clique->parent_.lock()->children_.remove(clique);

	  // orphan my children
		BOOST_FOREACH(sharedClique child, clique->children_)
	  	child->parent_ = typename BayesTree<Conditional>::Clique::weak_ptr();

	  BOOST_FOREACH(Index key, clique->separator_) {
			nodes_[key].reset();
	  }
	  const Clique& cliquer(*clique);
	  BOOST_FOREACH(const sharedConditional& conditional, cliquer) {
	    nodes_[conditional->key()].reset();
	  }
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
	BayesTree<Conditional>::BayesTree(const BayesNet<Conditional>& bayesNet, std::list<BayesTree<Conditional> > subtrees) {
		if (bayesNet.size() == 0)
			throw invalid_argument("BayesTree::insert: empty bayes net!");

		// get the roots of child subtrees and merge their nodes_
		list<sharedClique> childRoots;
		BOOST_FOREACH(const BayesTree<Conditional>& subtree, subtrees) {
			nodes_.insert(subtree.nodes_.begin(), subtree.nodes_.end());
			childRoots.push_back(subtree.root());
		}

		// create a new clique and add all the conditionals to the clique
		sharedClique new_clique;
		typename BayesNet<Conditional>::sharedConditional conditional;
		BOOST_REVERSE_FOREACH(conditional, bayesNet) {
			if (!new_clique.get())
				new_clique = addClique(conditional,childRoots);
			else
			  addToCliqueFront(conditional, new_clique);
		}

		root_ = new_clique;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::print(const string& s) const {
		if (root_.use_count() == 0) {
			printf("WARNING: BayesTree.print encountered a forest...\n");
			return;
		}
		cout << s << ": clique size == " << size() << ", node size == " << nodes_.size() <<  endl;
		if (nodes_.empty()) return;
		root_->printTree("");
	}

	/* ************************************************************************* */
	// binary predicate to test equality of a pair for use in equals
	template<class Conditional>
	bool check_sharedCliques(
			const typename BayesTree<Conditional>::sharedClique& v1,
			const typename BayesTree<Conditional>::sharedClique& v2
	) {
		return v1->equals(*v2);
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesTree<Conditional>::equals(const BayesTree<Conditional>& other,
			double tol) const {
		return size()==other.size() &&
				std::equal(nodes_.begin(), nodes_.end(), other.nodes_.begin(), &check_sharedCliques<Conditional>);
	}

	/* ************************************************************************* */
	template<class Conditional>
	template<class Container>
	inline Index BayesTree<Conditional>::findParentClique(const Container& parents) const {
	  typename Container::const_iterator lowestOrderedParent = min_element(parents.begin(), parents.end());
	  assert(lowestOrderedParent != parents.end());
	  return *lowestOrderedParent;

//		boost::optional<Index> parentCliqueRepresentative;
//		boost::optional<size_t> lowest;
//		BOOST_FOREACH(Index p, parents) {
//			size_t i = index(p);
//			if (!lowest || i<*lowest) {
//				lowest.reset(i);
//				parentCliqueRepresentative.reset(p);
//			}
//		}
//		if (!lowest) throw
//			invalid_argument("BayesTree::findParentClique: no parents given or key not present in index");
//		return *parentCliqueRepresentative;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::insert(const sharedConditional& conditional)
	{
	  static const bool debug = false;

		// get key and parents
		typename Conditional::Parents parents = conditional->parents(); // todo: const reference?

		if(debug) conditional->print("Adding conditional ");

		// if no parents, start a new root clique
		if (parents.empty()) {
		  if(debug) cout << "No parents so making root" << endl;
			root_ = addClique(conditional);
			return;
		}

		// otherwise, find the parent clique by using the index data structure
		// to find the lowest-ordered parent
		Index parentRepresentative = findParentClique(parents);
		if(debug) cout << "First-eliminated parent is " << parentRepresentative << endl;
		sharedClique parent_clique = (*this)[parentRepresentative];
		if(debug) parent_clique->print("Parent clique is ");

		// if the parents and parent clique have the same size, add to parent clique
		if (parent_clique->size() == size_t(parents.size())) {
		  if(debug) cout << "Adding to parent clique" << endl;
#ifndef NDEBUG
		  // Debug check that the parent keys of the new conditional match the keys
		  // currently in the clique.
//		  list<Index>::const_iterator parent = parents.begin();
//		  typename Clique::const_iterator cond = parent_clique->begin();
//		  while(parent != parents.end()) {
//		    assert(cond != parent_clique->end());
//		    assert(*parent == (*cond)->key());
//		    ++ parent;
//		    ++ cond;
//		  }
#endif
		  addToCliqueFront(conditional, parent_clique);
		} else {
		  if(debug) cout << "Starting new clique" << endl;
		  // otherwise, start a new clique and add it to the tree
		  addClique(conditional,parent_clique);
		}
	}

	/* ************************************************************************* */
	//TODO: remove this function after removing TSAM.cpp
	template<class Conditional>
	typename BayesTree<Conditional>::sharedClique BayesTree<Conditional>::insert(
			const BayesNet<Conditional>& bayesNet, list<sharedClique>& children, bool isRootClique)
	{
	  static const bool debug = false;

		if (bayesNet.size() == 0)
			throw invalid_argument("BayesTree::insert: empty bayes net!");

		// create a new clique and add all the conditionals to the clique
		sharedClique new_clique;
		typename BayesNet<Conditional>::sharedConditional conditional;
		BOOST_REVERSE_FOREACH(conditional, bayesNet) {
		  if(debug) conditional->print("Inserting conditional: ");
			if (!new_clique.get())
				new_clique = addClique(conditional,children);
			else
			  addToCliqueFront(conditional, new_clique);
		}

		if (isRootClique) root_ = new_clique;

		return new_clique;
	}

  /* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::fillNodesIndex(const sharedClique& subtree) {
	  // Add each frontal variable of this root node
	  BOOST_FOREACH(const typename Conditional::shared_ptr& cond, *subtree) { nodes_[cond->key()] = subtree; }
	  // Fill index for each child
	  BOOST_FOREACH(const typename BayesTree<Conditional>::sharedClique& child, subtree->children_) {
	    fillNodesIndex(child); }
	}

  /* ************************************************************************* */
	template<class Conditional>
	void BayesTree<Conditional>::insert(const sharedClique& subtree) {
	  if(subtree) {
	    // Find the parent clique of the new subtree.  By the running intersection
	    // property, those separator variables in the subtree that are ordered
	    // lower than the highest frontal variable of the subtree root will all
	    // appear in the separator of the subtree root.
	    if(subtree->separator_.empty()) {
	      assert(!root_);
	      root_ = subtree;
	    } else {
	      Index parentRepresentative = findParentClique(subtree->separator_);
	      sharedClique parent = (*this)[parentRepresentative];
	      parent->children_ += subtree;
	      subtree->parent_ = parent; // set new parent!
	    }

	    // Now fill in the nodes index
	    if(subtree->back()->key() > (nodes_.size() - 1))
	      nodes_.resize(subtree->back()->key() + 1);
	    fillNodesIndex(subtree);
	  }
	}

	/* ************************************************************************* */
	// First finds clique marginal then marginalizes that
	/* ************************************************************************* */
	template<class Conditional>
	template<class FactorGraph>
	FactorGraph
	BayesTree<Conditional>::marginal(Index key) const {

		// get clique containing key
		sharedClique clique = (*this)[key];

		// calculate or retrieve its marginal
		FactorGraph cliqueMarginal = clique->marginal<FactorGraph>(root_);

		// Reorder so that only the requested key is not eliminated
		typename FactorGraph::variableindex_type varIndex(cliqueMarginal);
		vector<Index> keyAsVector(1); keyAsVector[0] = key;
		Permutation toBack(Permutation::PushToBack(keyAsVector, varIndex.size()));
		Permutation::shared_ptr toBackInverse(toBack.inverse());
		varIndex.permute(toBack);
		BOOST_FOREACH(const typename FactorGraph::sharedFactor& factor, cliqueMarginal) {
		  factor->permuteWithInverse(*toBackInverse);
		}

		// partially eliminate, remaining factor graph is requested marginal
		Inference::EliminateUntil(cliqueMarginal, varIndex.size()-1, varIndex);
    BOOST_FOREACH(const typename FactorGraph::sharedFactor& factor, cliqueMarginal) {
      if(factor)
        factor->permuteWithInverse(toBack);
    }
		return cliqueMarginal;
	}

	/* ************************************************************************* */
	template<class Conditional>
	template<class FactorGraph>
	BayesNet<Conditional>
	BayesTree<Conditional>::marginalBayesNet(Index key) const {

		// calculate marginal as a factor graph
	  FactorGraph fg = this->marginal<FactorGraph>(key);

		// eliminate further to Bayes net
		return *Inference::Eliminate(fg);
	}

//	/* ************************************************************************* */
//	// Find two cliques, their joint, then marginalizes
//	/* ************************************************************************* */
//	template<class Conditional>
//	template<class Factor>
//	FactorGraph<Factor>
//	BayesTree<Conditional>::joint(Index key1, Index key2) const {
//
//		// get clique C1 and C2
//		sharedClique C1 = (*this)[key1], C2 = (*this)[key2];
//
//		// calculate joint
//		Ordering ord;
//		FactorGraph<Factor> p_C1C2;
//		boost::tie(p_C1C2,ord) = C1->joint<Factor>(C2,root_);
//
//		// create an ordering where both requested keys are not eliminated
//		ord.remove(key1);
//		ord.remove(key2);
//
//		// partially eliminate, remaining factor graph is requested joint
//		// TODO, make eliminate functional
//		eliminate<Factor,Conditional>(p_C1C2,ord);
//		return p_C1C2;
//	}

//	/* ************************************************************************* */
//	template<class Conditional>
//	template<class Factor>
//	BayesNet<Conditional>
//	BayesTree<Conditional>::jointBayesNet(Index key1, Index key2) const {
//
//		// calculate marginal as a factor graph
//	  FactorGraph<Factor> fg = this->joint<Factor>(key1,key2);
//
//		// eliminate further to Bayes net
//		Ordering ordering;
//		ordering += key1, key2;
//		return eliminate<Factor,Conditional>(fg,ordering);
//	}

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
			this->removePath(clique->parent_.lock(), bn, orphans);

			// add children to list of orphans (splice also removed them from clique->children_)
			orphans.splice (orphans.begin(), clique->children_);

			bn.push_back(*clique);

		}
	}

	/* ************************************************************************* */
	template<class Conditional>
  template<class Container>
  void BayesTree<Conditional>::removeTop(const Container& keys,
  		BayesNet<Conditional>& bn, typename BayesTree<Conditional>::Cliques& orphans) {

		// process each key of the new factor
	  BOOST_FOREACH(const Index& key, keys) {

	    // get the clique
	    if(key < nodes_.size()) {
	      const sharedClique& clique(nodes_[key]);
	      if(clique) {
	        // remove path from clique to root
	        this->removePath(clique, bn, orphans);
	      }
	    }
	  }
	}

/* ************************************************************************* */

}
/// namespace gtsam
