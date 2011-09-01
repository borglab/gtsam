/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/base/FastSet.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/inference.h>
#include <gtsam/inference/GenericSequentialSolver-inl.h>

#include <iostream>
#include <algorithm>

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/format.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <fstream>
using namespace boost::assign;
namespace lam = boost::lambda;

namespace gtsam {

	using namespace std;

  /* ************************************************************************* */
  template<class CONDITIONAL>
  void BayesTree<CONDITIONAL>::Clique::assertInvariants() const {
#ifndef NDEBUG
    // We rely on the keys being sorted
//    FastVector<Index> sortedUniqueKeys(conditional_->begin(), conditional_->end());
//    std::sort(sortedUniqueKeys.begin(), sortedUniqueKeys.end());
//    std::unique(sortedUniqueKeys.begin(), sortedUniqueKeys.end());
//    assert(sortedUniqueKeys.size() == conditional_->size() &&
//        std::equal(sortedUniqueKeys.begin(), sortedUniqueKeys.end(), conditional_->begin()));
#endif
  }

	/* ************************************************************************* */
	template<class CONDITIONAL>
	BayesTree<CONDITIONAL>::Clique::Clique(const sharedConditional& conditional) : conditional_(conditional) {
		assertInvariants();
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::Clique::print(const string& s) const {
		conditional_->print(s);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	size_t BayesTree<CONDITIONAL>::Clique::treeSize() const {
		size_t size = 1;
		BOOST_FOREACH(const shared_ptr& child, children_)
			size += child->treeSize();
		return size;
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::Clique::printTree(const string& indent) const {
		print(indent);
		BOOST_FOREACH(const shared_ptr& child, children_)
			child->printTree(indent+"  ");
	}

  /* ************************************************************************* */
  template<class CONDITIONAL>
  void BayesTree<CONDITIONAL>::Clique::permuteWithInverse(const Permutation& inversePermutation) {
    conditional_->permuteWithInverse(inversePermutation);
    if(cachedFactor_) cachedFactor_->permuteWithInverse(inversePermutation);
    BOOST_FOREACH(const shared_ptr& child, children_) {
      child->permuteWithInverse(inversePermutation);
    }
    assertInvariants();
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  bool BayesTree<CONDITIONAL>::Clique::permuteSeparatorWithInverse(const Permutation& inversePermutation) {
    bool changed = conditional_->permuteSeparatorWithInverse(inversePermutation);
#ifndef NDEBUG
    if(!changed) {
      BOOST_FOREACH(Index& separatorKey, conditional_->parents()) { assert(separatorKey == inversePermutation[separatorKey]); }
      BOOST_FOREACH(const shared_ptr& child, children_) {
        assert(child->permuteSeparatorWithInverse(inversePermutation) == false);
      }
    }
#endif
    if(changed) {
      if(cachedFactor_) cachedFactor_->permuteWithInverse(inversePermutation);
      BOOST_FOREACH(const shared_ptr& child, children_) {
        (void)child->permuteSeparatorWithInverse(inversePermutation);
      }
    }
    assertInvariants();
    return changed;
  }

	/* ************************************************************************* */
	template<class CONDITIONAL>
	typename BayesTree<CONDITIONAL>::CliqueData
	BayesTree<CONDITIONAL>::getCliqueData() const {
		CliqueData data;
		getCliqueData(data, root_);
		return data;
	}

	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::getCliqueData(CliqueData& data,
			BayesTree<CONDITIONAL>::sharedClique clique) const {
		data.conditionalSizes.push_back((*clique)->nrFrontals());
		data.separatorSizes.push_back((*clique)->nrParents());
		BOOST_FOREACH(sharedClique c, clique->children_) {
			getCliqueData(data, c);
		}
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::saveGraph(const std::string &s) const {
		if (!root_.get()) throw invalid_argument("the root of bayes tree has not been initialized!");
		ofstream of(s.c_str());
		of<< "digraph G{\n";
		saveGraph(of, root_);
		of<<"}";
		of.close();
	}

	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::saveGraph(ostream &s,
			BayesTree<CONDITIONAL>::sharedClique clique,
			int parentnum) const {
		static int num = 0;
		bool first = true;
		std::stringstream out;
		out << num;
		string parent = out.str();
		parent += "[label=\"";

		BOOST_FOREACH(boost::shared_ptr<CONDITIONAL> c, clique->conditionals_) {
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


	template<class CONDITIONAL>
	typename BayesTree<CONDITIONAL>::CliqueStats
	BayesTree<CONDITIONAL>::CliqueData::getStats() const {
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
	/* ************************************************************************* */
	template<class CONDITIONAL>
	BayesNet<CONDITIONAL> BayesTree<CONDITIONAL>::Clique::shortcut(shared_ptr R,
			Eliminate function) {

	  static const bool debug = false;

		// A first base case is when this clique or its parent is the root,
		// in which case we return an empty Bayes net.

	  sharedClique parent(parent_.lock());

		if (R.get()==this || parent==R) {
			BayesNet<CONDITIONAL> empty;
			return empty;
		}

		// The root conditional
		FactorGraph<FactorType> p_R(BayesNet<CONDITIONAL>(R->conditional()));

		// The parent clique has a CONDITIONAL for each frontal node in Fp
		// so we can obtain P(Fp|Sp) in factor graph form
		FactorGraph<FactorType> p_Fp_Sp(BayesNet<CONDITIONAL>(parent->conditional()));

		// If not the base case, obtain the parent shortcut P(Sp|R) as factors
		FactorGraph<FactorType> p_Sp_R(parent->shortcut(R, function));

		// now combine P(Cp|R) = P(Fp|Sp) * P(Sp|R)
		FactorGraph<FactorType> p_Cp_R;
		p_Cp_R.push_back(p_R);
		p_Cp_R.push_back(p_Fp_Sp);
		p_Cp_R.push_back(p_Sp_R);

		// Eliminate into a Bayes net with ordering designed to integrate out
		// any variables not in *our* separator. Variables to integrate out must be
		// eliminated first hence the desired ordering is [Cp\S S].
		// However, an added wrinkle is that Cp might overlap with the root.
		// Keys corresponding to the root should not be added to the ordering at all.

		if(debug) {
		  p_R.print("p_R: ");
		  p_Fp_Sp.print("p_Fp_Sp: ");
		  p_Sp_R.print("p_Sp_R: ");
		}

		// We want to factor into a conditional of the clique variables given the
		// root and the marginal on the root, integrating out all other variables.
		// The integrands include any parents of this clique and the variables of
		// the parent clique.
		FastSet<Index> variablesAtBack;
		FastSet<Index> separator;
		size_t uniqueRootVariables = 0;
		BOOST_FOREACH(const Index separatorIndex, this->conditional()->parents()) {
		  variablesAtBack.insert(separatorIndex);
		  separator.insert(separatorIndex);
		  if(debug) cout << "At back (this): " << separatorIndex << endl;
		}
		BOOST_FOREACH(const Index key, R->conditional()->keys()) {
		  if(variablesAtBack.insert(key).second)
		    ++ uniqueRootVariables;
		  if(debug) cout << "At back (root): " << key << endl;
		}

		Permutation toBack = Permutation::PushToBack(
		    vector<Index>(variablesAtBack.begin(), variablesAtBack.end()),
		    R->conditional()->lastFrontalKey() + 1);
    Permutation::shared_ptr toBackInverse(toBack.inverse());
    BOOST_FOREACH(const typename FactorType::shared_ptr& factor, p_Cp_R) {
      factor->permuteWithInverse(*toBackInverse); }
		typename BayesNet<CONDITIONAL>::shared_ptr eliminated(EliminationTree<
				FactorType>::Create(p_Cp_R)->eliminate(function));

		// Take only the conditionals for p(S|R).  We check for each variable being
		// in the separator set because if some separator variables overlap with
		// root variables, we cannot rely on the number of root variables, and also
		// want to include those variables in the conditional.
		BayesNet<CONDITIONAL> p_S_R;
		BOOST_REVERSE_FOREACH(typename CONDITIONAL::shared_ptr conditional, *eliminated) {
		  assert(conditional->nrFrontals() == 1);
		  if(separator.find(toBack[conditional->firstFrontalKey()]) != separator.end()) {
		    if(debug)
		      conditional->print("Taking C|R conditional: ");
		    p_S_R.push_front(conditional);
		  }
		  if(p_S_R.size() == separator.size())
		    break;
		}

		// Undo the permutation
		if(debug) toBack.print("toBack: ");
		p_S_R.permuteWithInverse(toBack);

		// return the parent shortcut P(Sp|R)
    assertInvariants();
		return p_S_R;
	}

	/* ************************************************************************* */
	// P(C) = \int_R P(F|S) P(S|R) P(R)
	// TODO: Maybe we should integrate given parent marginal P(Cp),
	// \int(Cp\S) P(F|S)P(S|Cp)P(Cp)
	// Because the root clique could be very big.
	/* ************************************************************************* */
	template<class CONDITIONAL>
	FactorGraph<typename CONDITIONAL::FactorType> BayesTree<CONDITIONAL>::Clique::marginal(
			shared_ptr R, Eliminate function) {
		// If we are the root, just return this root
		// NOTE: immediately cast to a factor graph
	  BayesNet<CONDITIONAL> bn(R->conditional());
		if (R.get()==this) return bn;

		// Combine P(F|S), P(S|R), and P(R)
		BayesNet<CONDITIONAL> p_FSR = this->shortcut(R, function);
		p_FSR.push_front(this->conditional());
		p_FSR.push_back(R->conditional());

    assertInvariants();
		GenericSequentialSolver<FactorType> solver(p_FSR);
		return *solver.jointFactorGraph(conditional_->keys(), function);
	}

	/* ************************************************************************* */
	// P(C1,C2) = \int_R P(F1|S1) P(S1|R) P(F2|S1) P(S2|R) P(R)
	/* ************************************************************************* */
	template<class CONDITIONAL>
	FactorGraph<typename CONDITIONAL::FactorType> BayesTree<CONDITIONAL>::Clique::joint(
			shared_ptr C2, shared_ptr R, Eliminate function) {
		// For now, assume neither is the root

		// Combine P(F1|S1), P(S1|R), P(F2|S2), P(S2|R), and P(R)
		FactorGraph<FactorType> joint;
		if (!isRoot()) joint.push_back(this->conditional()->toFactor()); // P(F1|S1)
		if (!isRoot()) joint.push_back(shortcut(R, function)); // P(S1|R)
		if (!C2->isRoot()) joint.push_back(C2->conditional()->toFactor()); // P(F2|S2)
		if (!C2->isRoot()) joint.push_back(C2->shortcut(R, function)); // P(S2|R)
		joint.push_back(R->conditional()->toFactor()); // P(R)

		// Find the keys of both C1 and C2
		vector<Index> keys1(conditional_->keys());
		vector<Index> keys2(C2->conditional_->keys());
		FastSet<Index> keys12;
		keys12.insert(keys1.begin(), keys1.end());
		keys12.insert(keys2.begin(), keys2.end());

		// Calculate the marginal
		vector<Index> keys12vector; keys12vector.reserve(keys12.size());
		keys12vector.insert(keys12vector.begin(), keys12.begin(), keys12.end());
    assertInvariants();
    GenericSequentialSolver<FactorType> solver(joint);
		return *solver.jointFactorGraph(keys12vector, function);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::Cliques::print(const std::string& s) const {
		cout << s << ":\n";
		BOOST_FOREACH(sharedClique clique, *this)
				clique->printTree();
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	bool BayesTree<CONDITIONAL>::Cliques::equals(const Cliques& other, double tol) const {
		return other == *this;
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	typename BayesTree<CONDITIONAL>::sharedClique BayesTree<CONDITIONAL>::addClique(
	    const sharedConditional& conditional, sharedClique parent_clique) {
		sharedClique new_clique(new Clique(conditional));
    nodes_.resize(std::max(conditional->lastFrontalKey()+1, nodes_.size()));
    BOOST_FOREACH(Index key, conditional->frontals())
      nodes_[key] = new_clique;
		if (parent_clique != NULL) {
			new_clique->parent_ = parent_clique;
			parent_clique->children_.push_back(new_clique);
		}
    new_clique->assertInvariants();
		return new_clique;
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	typename BayesTree<CONDITIONAL>::sharedClique BayesTree<CONDITIONAL>::addClique(
	    const sharedConditional& conditional, list<sharedClique>& child_cliques) {
		sharedClique new_clique(new Clique(conditional));
		nodes_.resize(std::max(conditional->lastFrontalKey()+1, nodes_.size()));
		BOOST_FOREACH(Index key, conditional->frontals())
		  nodes_[key] = new_clique;
		new_clique->children_ = child_cliques;
		BOOST_FOREACH(sharedClique& child, child_cliques)
			child->parent_ = new_clique;
		new_clique->assertInvariants();
		return new_clique;
	}

  /* ************************************************************************* */
	template<class CONDITIONAL>
	inline void BayesTree<CONDITIONAL>::addToCliqueFront(BayesTree<CONDITIONAL>& bayesTree, const sharedConditional& conditional, const sharedClique& clique) {
	  static const bool debug = false;
#ifndef NDEBUG
	  // Debug check to make sure the conditional variable is ordered lower than
	  // its parents and that all of its parents are present either in this
	  // clique or its separator.
	  BOOST_FOREACH(Index parent, conditional->parents()) {
	    assert(parent > conditional->lastFrontalKey());
	    const Clique& cliquer(*clique);
	    assert(find(cliquer->begin(), cliquer->end(), parent) != cliquer->end());
	  }
#endif
	  if(debug) conditional->print("Adding conditional ");
	  if(debug) clique->print("To clique ");
	  Index key = conditional->lastFrontalKey();
	  bayesTree.nodes_.resize(std::max(key+1, bayesTree.nodes_.size()));
	  bayesTree.nodes_[key] = clique;
	  FastVector<Index> newKeys((*clique)->size() + 1);
	  newKeys[0] = key;
	  std::copy((*clique)->begin(), (*clique)->end(), newKeys.begin()+1);
	  clique->conditional_ = CONDITIONAL::FromKeys(newKeys, (*clique)->nrFrontals() + 1);
//	  clique->conditional_->addFrontal(conditional);
	  if(debug) clique->print("Expanded clique is ");
	  clique->assertInvariants();
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::removeClique(sharedClique clique) {

		if (clique->isRoot())
			root_.reset();
		else // detach clique from parent
	    clique->parent_.lock()->children_.remove(clique);

	  // orphan my children
		BOOST_FOREACH(sharedClique child, clique->children_)
	  	child->parent_ = typename BayesTree<CONDITIONAL>::Clique::weak_ptr();

	  BOOST_FOREACH(Index key, (*clique->conditional())) {
			nodes_[key].reset();
	  }
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	BayesTree<CONDITIONAL>::BayesTree() {
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	BayesTree<CONDITIONAL>::BayesTree(const BayesNet<CONDITIONAL>& bayesNet) {
		typename BayesNet<CONDITIONAL>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			insert(*this, *rit);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	BayesTree<CONDITIONAL>::BayesTree(const BayesNet<CONDITIONAL>& bayesNet, std::list<BayesTree<CONDITIONAL> > subtrees) {
		if (bayesNet.size() == 0)
			throw invalid_argument("BayesTree::insert: empty bayes net!");

		// get the roots of child subtrees and merge their nodes_
		list<sharedClique> childRoots;
		BOOST_FOREACH(const BayesTree<CONDITIONAL>& subtree, subtrees) {
			nodes_.insert(subtree.nodes_.begin(), subtree.nodes_.end());
			childRoots.push_back(subtree.root());
		}

		// create a new clique and add all the conditionals to the clique
		sharedClique new_clique;
		typename BayesNet<CONDITIONAL>::sharedConditional conditional;
		BOOST_REVERSE_FOREACH(conditional, bayesNet) {
			if (!new_clique.get())
				new_clique = addClique(conditional,childRoots);
			else
			  addToCliqueFront(*this, conditional, new_clique);
		}

		root_ = new_clique;
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::print(const string& s) const {
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
	template<class CONDITIONAL>
	bool check_sharedCliques(
			const typename BayesTree<CONDITIONAL>::sharedClique& v1,
			const typename BayesTree<CONDITIONAL>::sharedClique& v2
	) {
		return v1->equals(*v2);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	bool BayesTree<CONDITIONAL>::equals(const BayesTree<CONDITIONAL>& other,
			double tol) const {
		return size()==other.size() &&
				std::equal(nodes_.begin(), nodes_.end(), other.nodes_.begin(), &check_sharedCliques<CONDITIONAL>);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	template<class CONTAINER>
	inline Index BayesTree<CONDITIONAL>::findParentClique(const CONTAINER& parents) const {
	  typename CONTAINER::const_iterator lowestOrderedParent = min_element(parents.begin(), parents.end());
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
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::insert(BayesTree<CONDITIONAL>& bayesTree, const sharedConditional& conditional)
	{
	  static const bool debug = false;

		// get key and parents
		const typename CONDITIONAL::Parents& parents = conditional->parents();

		if(debug) conditional->print("Adding conditional ");

		// if no parents, start a new root clique
		if (parents.empty()) {
		  if(debug) cout << "No parents so making root" << endl;
		  bayesTree.root_ = bayesTree.addClique(conditional);
			return;
		}

		// otherwise, find the parent clique by using the index data structure
		// to find the lowest-ordered parent
		Index parentRepresentative = bayesTree.findParentClique(parents);
		if(debug) cout << "First-eliminated parent is " << parentRepresentative << ", have " << bayesTree.nodes_.size() << " nodes." << endl;
		sharedClique parent_clique = bayesTree[parentRepresentative];
		if(debug) parent_clique->print("Parent clique is ");

		// if the parents and parent clique have the same size, add to parent clique
		if ((*parent_clique)->size() == size_t(parents.size())) {
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
		  addToCliqueFront(bayesTree, conditional, parent_clique);
		} else {
		  if(debug) cout << "Starting new clique" << endl;
		  // otherwise, start a new clique and add it to the tree
		  bayesTree.addClique(conditional,parent_clique);
		}
	}

	/* ************************************************************************* */
	//TODO: remove this function after removing TSAM.cpp
	template<class CONDITIONAL>
	typename BayesTree<CONDITIONAL>::sharedClique BayesTree<CONDITIONAL>::insert(
	    const sharedConditional& clique, list<sharedClique>& children, bool isRootClique) {

		if (clique->nrFrontals() == 0)
			throw invalid_argument("BayesTree::insert: empty clique!");

		// create a new clique and add all the conditionals to the clique
		sharedClique new_clique = addClique(clique, children);
		if (isRootClique) root_ = new_clique;

		return new_clique;
	}

  /* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::fillNodesIndex(const sharedClique& subtree) {
	  // Add each frontal variable of this root node
	  BOOST_FOREACH(const Index& key, subtree->conditional()->frontals()) { nodes_[key] = subtree; }
	  // Fill index for each child
	  BOOST_FOREACH(const typename BayesTree<CONDITIONAL>::sharedClique& child, subtree->children_) {
	    fillNodesIndex(child); }
	}

  /* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::insert(const sharedClique& subtree) {
	  if(subtree) {
	    // Find the parent clique of the new subtree.  By the running intersection
	    // property, those separator variables in the subtree that are ordered
	    // lower than the highest frontal variable of the subtree root will all
	    // appear in the separator of the subtree root.
	    if(subtree->conditional()->parents().empty()) {
	      assert(!root_);
	      root_ = subtree;
	    } else {
	      Index parentRepresentative = findParentClique(subtree->conditional()->parents());
	      sharedClique parent = (*this)[parentRepresentative];
	      parent->children_ += subtree;
	      subtree->parent_ = parent; // set new parent!
	    }

	    // Now fill in the nodes index
	    if(nodes_.size() == 0 ||
	        *std::max_element(subtree->conditional()->beginFrontals(), subtree->conditional()->endFrontals()) > (nodes_.size() - 1)) {
	      nodes_.resize(subtree->conditional()->lastFrontalKey() + 1);
	    }
	    fillNodesIndex(subtree);
	  }
	}

	/* ************************************************************************* */
	// First finds clique marginal then marginalizes that
	/* ************************************************************************* */
	template<class CONDITIONAL>
	typename CONDITIONAL::FactorType::shared_ptr BayesTree<CONDITIONAL>::marginalFactor(
			Index key, Eliminate function) const {

		// get clique containing key
		sharedClique clique = (*this)[key];

		// calculate or retrieve its marginal
		FactorGraph<FactorType> cliqueMarginal = clique->marginal(root_,function);

		return GenericSequentialSolver<FactorType>(cliqueMarginal).marginalFactor(
				key, function);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	typename BayesNet<CONDITIONAL>::shared_ptr BayesTree<CONDITIONAL>::marginalBayesNet(
			Index key, Eliminate function) const {

	  // calculate marginal as a factor graph
	  FactorGraph<FactorType> fg;
	  fg.push_back(this->marginalFactor(key,function));

		// eliminate factor graph marginal to a Bayes net
		return GenericSequentialSolver<FactorType>(fg).eliminate(function);
	}

	/* ************************************************************************* */
	// Find two cliques, their joint, then marginalizes
	/* ************************************************************************* */
	template<class CONDITIONAL>
	typename FactorGraph<typename CONDITIONAL::FactorType>::shared_ptr BayesTree<
			CONDITIONAL>::joint(Index key1, Index key2, Eliminate function) const {

		// get clique C1 and C2
		sharedClique C1 = (*this)[key1], C2 = (*this)[key2];

		// calculate joint
		FactorGraph<FactorType> p_C1C2(C1->joint(C2, root_, function));

		// eliminate remaining factor graph to get requested joint
		vector<Index> key12(2); key12[0] = key1; key12[1] = key2;
		GenericSequentialSolver<FactorType> solver(p_C1C2);
		return solver.jointFactorGraph(key12,function);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	typename BayesNet<CONDITIONAL>::shared_ptr BayesTree<CONDITIONAL>::jointBayesNet(
			Index key1, Index key2, Eliminate function) const {

		// eliminate factor graph marginal to a Bayes net
		return GenericSequentialSolver<FactorType> (
				*this->joint(key1, key2, function)).eliminate(function);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::clear() {
		// Remove all nodes and clear the root pointer
		nodes_.clear();
		root_.reset();
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesTree<CONDITIONAL>::removePath(sharedClique clique,
			BayesNet<CONDITIONAL>& bn, typename BayesTree<CONDITIONAL>::Cliques& orphans) {

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

			bn.push_back(clique->conditional());

		}
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
  template<class CONTAINER>
  void BayesTree<CONDITIONAL>::removeTop(const CONTAINER& keys,
  		BayesNet<CONDITIONAL>& bn, typename BayesTree<CONDITIONAL>::Cliques& orphans) {

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
