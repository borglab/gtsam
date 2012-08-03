/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeCliqueBase-inl.h
 * @brief   Base class for cliques of a BayesTree
 * @author  Richard Roberts and Frank Dellaert
 */

#pragma once

#include <gtsam/inference/GenericSequentialSolver.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED,CONDITIONAL>::assertInvariants() const {
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
  template<class DERIVED, class CONDITIONAL>
  BayesTreeCliqueBase<DERIVED,CONDITIONAL>::BayesTreeCliqueBase(const sharedConditional& conditional) :
  conditional_(conditional) {
    assertInvariants();
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  BayesTreeCliqueBase<DERIVED,CONDITIONAL>::BayesTreeCliqueBase(const std::pair<sharedConditional, boost::shared_ptr<typename ConditionalType::FactorType> >& result) :
  conditional_(result.first) {
    assertInvariants();
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED,CONDITIONAL>::print(const std::string& s, const IndexFormatter& indexFormatter) const {
    conditional_->print(s, indexFormatter);
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  size_t BayesTreeCliqueBase<DERIVED,CONDITIONAL>::treeSize() const {
    size_t size = 1;
    BOOST_FOREACH(const derived_ptr& child, children_)
      size += child->treeSize();
    return size;
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED,CONDITIONAL>::printTree(const std::string& indent, const IndexFormatter& indexFormatter) const {
    asDerived(this)->print(indent, indexFormatter);
    BOOST_FOREACH(const derived_ptr& child, children_)
      child->printTree(indent+"  ", indexFormatter);
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED,CONDITIONAL>::permuteWithInverse(const Permutation& inversePermutation) {
    conditional_->permuteWithInverse(inversePermutation);
    BOOST_FOREACH(const derived_ptr& child, children_) {
      child->permuteWithInverse(inversePermutation);
    }
    assertInvariants();
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  bool BayesTreeCliqueBase<DERIVED,CONDITIONAL>::permuteSeparatorWithInverse(const Permutation& inversePermutation) {
    bool changed = conditional_->permuteSeparatorWithInverse(inversePermutation);
#ifndef NDEBUG
    if(!changed) {
      BOOST_FOREACH(Index& separatorKey, conditional_->parents()) { assert(separatorKey == inversePermutation[separatorKey]); }
      BOOST_FOREACH(const derived_ptr& child, children_) {
        assert(child->permuteSeparatorWithInverse(inversePermutation) == false);
      }
    }
#endif
    if(changed) {
      BOOST_FOREACH(const derived_ptr& child, children_) {
        (void)child->permuteSeparatorWithInverse(inversePermutation);
      }
    }
    assertInvariants();
    return changed;
  }

  /* ************************************************************************* */
  // The shortcut density is a conditional P(S|R) of the separator of this
  // clique on the root. We can compute it recursively from the parent shortcut
  // P(Sp|R) as \int P(Fp|Sp) P(Sp|R), where Fp are the frontal nodes in p
  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  BayesNet<CONDITIONAL> BayesTreeCliqueBase<DERIVED, CONDITIONAL>::shortcut(
  		derived_ptr R, Eliminate function) const{

  	static const bool debug = false;

  	BayesNet<ConditionalType> p_S_R;	//shortcut P(S|R) This is empty now

  	//Check if the ShortCut already exists
  	if(!cachedShortcut_){

  		// A first base case is when this clique or its parent is the root,
  		// in which case we return an empty Bayes net.

  		derived_ptr parent(parent_.lock());
  		if (R.get() != this && parent != R) {

  			// The root conditional
  			FactorGraph<FactorType> p_R(BayesNet<ConditionalType>(R->conditional()));

  			// The parent clique has a ConditionalType for each frontal node in Fp
  			// so we can obtain P(Fp|Sp) in factor graph form
  			FactorGraph<FactorType> p_Fp_Sp(BayesNet<ConditionalType>(parent->conditional()));

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
  				if(debug) std::cout << "At back (this): " << separatorIndex << std::endl;
  			}
  			BOOST_FOREACH(const Index key, R->conditional()->keys()) {
  				if(variablesAtBack.insert(key).second)
  					++ uniqueRootVariables;
  				if(debug) std::cout << "At back (root): " << key << std::endl;
  			}

  			Permutation toBack = Permutation::PushToBack(
  					std::vector<Index>(variablesAtBack.begin(), variablesAtBack.end()),
  					R->conditional()->lastFrontalKey() + 1);
  			Permutation::shared_ptr toBackInverse(toBack.inverse());
  			BOOST_FOREACH(const typename FactorType::shared_ptr& factor, p_Cp_R) {
  				factor->permuteWithInverse(*toBackInverse); }
  			typename BayesNet<ConditionalType>::shared_ptr eliminated(EliminationTree<
  					FactorType>::Create(p_Cp_R)->eliminate(function));

  			// Take only the conditionals for p(S|R).  We check for each variable being
  			// in the separator set because if some separator variables overlap with
  			// root variables, we cannot rely on the number of root variables, and also
  			// want to include those variables in the conditional.
  			BOOST_REVERSE_FOREACH(typename ConditionalType::shared_ptr conditional, *eliminated) {
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
  		}

  		cachedShortcut_ = p_S_R;
  	}
  	else
  		p_S_R = *cachedShortcut_;	// return the cached version

  	assertInvariants();

  	// return the shortcut P(S|R)
  	return p_S_R;
  }

  /* ************************************************************************* */
  // P(C) = \int_R P(F|S) P(S|R) P(R)
  // TODO: Maybe we should integrate given parent marginal P(Cp),
  // \int(Cp\S) P(F|S)P(S|Cp)P(Cp)
  // Because the root clique could be very big.
  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  FactorGraph<typename BayesTreeCliqueBase<DERIVED,CONDITIONAL>::FactorType> BayesTreeCliqueBase<DERIVED,CONDITIONAL>::marginal(
      derived_ptr R, Eliminate function) const{
    // If we are the root, just return this root
    // NOTE: immediately cast to a factor graph
    BayesNet<ConditionalType> bn(R->conditional());
    if (R.get()==this) return bn;

    // Combine P(F|S), P(S|R), and P(R)
    BayesNet<ConditionalType> p_FSR = this->shortcut(R, function);
    p_FSR.push_front(this->conditional());
    p_FSR.push_back(R->conditional());

    assertInvariants();
    GenericSequentialSolver<FactorType> solver(p_FSR);
    return *solver.jointFactorGraph(conditional_->keys(), function);
  }

  /* ************************************************************************* */
  // P(C1,C2) = \int_R P(F1|S1) P(S1|R) P(F2|S1) P(S2|R) P(R)
  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  FactorGraph<typename BayesTreeCliqueBase<DERIVED,CONDITIONAL>::FactorType> BayesTreeCliqueBase<DERIVED,CONDITIONAL>::joint(
      derived_ptr C2, derived_ptr R, Eliminate function) const {
    // For now, assume neither is the root

    // Combine P(F1|S1), P(S1|R), P(F2|S2), P(S2|R), and P(R)
    FactorGraph<FactorType> joint;
    if (!isRoot()) joint.push_back(this->conditional()->toFactor()); // P(F1|S1)
    if (!isRoot()) joint.push_back(shortcut(R, function)); // P(S1|R)
    if (!C2->isRoot()) joint.push_back(C2->conditional()->toFactor()); // P(F2|S2)
    if (!C2->isRoot()) joint.push_back(C2->shortcut(R, function)); // P(S2|R)
    joint.push_back(R->conditional()->toFactor()); // P(R)

    // Find the keys of both C1 and C2
    std::vector<Index> keys1(conditional_->keys());
    std::vector<Index> keys2(C2->conditional_->keys());
    FastSet<Index> keys12;
    keys12.insert(keys1.begin(), keys1.end());
    keys12.insert(keys2.begin(), keys2.end());

    // Calculate the marginal
    std::vector<Index> keys12vector; keys12vector.reserve(keys12.size());
    keys12vector.insert(keys12vector.begin(), keys12.begin(), keys12.end());
    assertInvariants();
    GenericSequentialSolver<FactorType> solver(joint);
    return *solver.jointFactorGraph(keys12vector, function);
  }

	/* ************************************************************************* */
	template<class DERIVED, class CONDITIONAL>
	void BayesTreeCliqueBase<DERIVED, CONDITIONAL>::deleteCachedShorcuts() {

	  // When a shortcut is requested, all of the shortcuts between it and the
	  // root are also generated. So, if this clique's cached shortcut is set,
	  // recursively call over all child cliques. Otherwise, it is unnecessary.
	  if(cachedShortcut_) {
	    BOOST_FOREACH(derived_ptr& child, children_) {
	      child->deleteCachedShorcuts();
	    }

	    //Delete CachedShortcut for this clique
	    this->resetCachedShortcut();
	  }

	}

}
