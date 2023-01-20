/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeCliqueBase-inst.h
 * @brief   Base class for cliques of a BayesTree
 * @author  Richard Roberts and Frank Dellaert
 */

#pragma once

#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/base/timing.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  void BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::setEliminationResult(
    const typename FactorGraphType::EliminationResult& eliminationResult)
  {
    conditional_ = eliminationResult.first;
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  bool BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::equals(
    const DERIVED& other, double tol) const
  {
    return (!conditional_ && !other.conditional())
      || conditional_->equals(*other.conditional(), tol);
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  KeyVector
    BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::separator_setminus_B(const derived_ptr& B) const
  {
    KeySet p_F_S_parents(this->conditional()->beginParents(), this->conditional()->endParents());
    KeySet indicesB(B->conditional()->begin(), B->conditional()->end());
    KeyVector S_setminus_B;
    std::set_difference(p_F_S_parents.begin(), p_F_S_parents.end(),
      indicesB.begin(), indicesB.end(), back_inserter(S_setminus_B));
    return S_setminus_B;
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  KeyVector BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::shortcut_indices(
    const derived_ptr& B, const FactorGraphType& p_Cp_B) const
  {
    gttic(shortcut_indices);
    KeySet allKeys = p_Cp_B.keys();
    KeySet indicesB(B->conditional()->begin(), B->conditional()->end());
    KeyVector S_setminus_B = separator_setminus_B(B);
    KeyVector keep;
    // keep = S\B intersect allKeys (S_setminus_B is already sorted)
    std::set_intersection(S_setminus_B.begin(), S_setminus_B.end(), //
      allKeys.begin(), allKeys.end(), back_inserter(keep));
    // keep += B intersect allKeys
    std::set_intersection(indicesB.begin(), indicesB.end(), //
      allKeys.begin(), allKeys.end(), back_inserter(keep));
    return keep;
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  void BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::print(
    const std::string& s, const KeyFormatter& keyFormatter) const
  {
    conditional_->print(s, keyFormatter);
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  size_t BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::treeSize() const {
    size_t size = 1;
    for(const derived_ptr& child: children)
      size += child->treeSize();
    return size;
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  size_t BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::numCachedSeparatorMarginals() const
  {
    std::lock_guard<std::mutex> marginalLock(cachedSeparatorMarginalMutex_);
    if (!cachedSeparatorMarginal_)
      return 0;

    size_t subtree_count = 1;
    for(const derived_ptr& child: children)
      subtree_count += child->numCachedSeparatorMarginals();

    return subtree_count;
  }

  /* ************************************************************************* */
  // The shortcut density is a conditional P(S|R) of the separator of this
  // clique on the root. We can compute it recursively from the parent shortcut
  // P(Sp|R) as \int P(Fp|Sp) P(Sp|R), where Fp are the frontal nodes in p
  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  typename BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::BayesNetType
    BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::shortcut(const derived_ptr& B, Eliminate function) const
  {
    gttic(BayesTreeCliqueBase_shortcut);
    // We only calculate the shortcut when this clique is not B
    // and when the S\B is not empty
    KeyVector S_setminus_B = separator_setminus_B(B);
    if (!parent_.expired() /*(if we're not the root)*/ && !S_setminus_B.empty())
    {
      // Obtain P(Cp||B) = P(Fp|Sp) * P(Sp||B) as a factor graph
      derived_ptr parent(parent_.lock());
      gttoc(BayesTreeCliqueBase_shortcut);
      FactorGraphType p_Cp_B(parent->shortcut(B, function)); // P(Sp||B)
      gttic(BayesTreeCliqueBase_shortcut);
      p_Cp_B += parent->conditional_; // P(Fp|Sp)

      // Determine the variables we want to keepSet, S union B
      KeyVector keep = shortcut_indices(B, p_Cp_B);

      // Marginalize out everything except S union B
      boost::shared_ptr<FactorGraphType> p_S_B = p_Cp_B.marginal(keep, function);
      return *p_S_B->eliminatePartialSequential(S_setminus_B, function).first;
    }
    else
    {
      return BayesNetType();
    }
  }

  /* *********************************************************************** */
  // separator marginal, uses separator marginal of parent recursively
  // P(C) = P(F|S) P(S)
  /* *********************************************************************** */
  template <class DERIVED, class FACTORGRAPH>
  typename BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::FactorGraphType
  BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::separatorMarginal(
      Eliminate function) const {
    std::lock_guard<std::mutex> marginalLock(cachedSeparatorMarginalMutex_);
    gttic(BayesTreeCliqueBase_separatorMarginal);
    // Check if the Separator marginal was already calculated
    if (!cachedSeparatorMarginal_) {
      gttic(BayesTreeCliqueBase_separatorMarginal_cachemiss);

      // If this is the root, there is no separator
      if (parent_.expired() /*(if we're the root)*/) {
        // we are root, return empty
        FactorGraphType empty;
        cachedSeparatorMarginal_ = empty;
      } else {
        // Flatten recursion in timing outline
        gttoc(BayesTreeCliqueBase_separatorMarginal_cachemiss);
        gttoc(BayesTreeCliqueBase_separatorMarginal);

        // Obtain P(S) = \int P(Cp) = \int P(Fp|Sp) P(Sp)
        // initialize P(Cp) with the parent separator marginal
        derived_ptr parent(parent_.lock());
        FactorGraphType p_Cp(parent->separatorMarginal(function));  // P(Sp)

        gttic(BayesTreeCliqueBase_separatorMarginal);
        gttic(BayesTreeCliqueBase_separatorMarginal_cachemiss);

        // now add the parent conditional
        p_Cp += parent->conditional_;  // P(Fp|Sp)

        // The variables we want to keepSet are exactly the ones in S
        KeyVector indicesS(this->conditional()->beginParents(),
                           this->conditional()->endParents());
        auto separatorMarginal =
            p_Cp.marginalMultifrontalBayesNet(Ordering(indicesS), function);
        cachedSeparatorMarginal_.reset(*separatorMarginal);
      }
    }

    // return the shortcut P(S||B)
    return *cachedSeparatorMarginal_;  // return the cached version
  }

  /* *********************************************************************** */
  // marginal2, uses separator marginal of parent
  // P(C) = P(F|S) P(S)
  /* *********************************************************************** */
  template <class DERIVED, class FACTORGRAPH>
  typename BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::FactorGraphType
  BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::marginal2(
      Eliminate function) const {
    gttic(BayesTreeCliqueBase_marginal2);
    // initialize with separator marginal P(S)
    FactorGraphType p_C = this->separatorMarginal(function);
    // add the conditional P(F|S)
    p_C += boost::shared_ptr<FactorType>(this->conditional_);
    return p_C;
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  void BayesTreeCliqueBase<DERIVED, FACTORGRAPH>::deleteCachedShortcuts() {

    // When a shortcut is requested, all of the shortcuts between it and the
    // root are also generated. So, if this clique's cached shortcut is set,
    // recursively call over all child cliques. Otherwise, it is unnecessary.
    
    std::lock_guard<std::mutex> marginalLock(cachedSeparatorMarginalMutex_);
    if (cachedSeparatorMarginal_) {
      for(derived_ptr& child: children) {
        child->deleteCachedShortcuts();
      }

      //Delete CachedShortcut for this clique
      cachedSeparatorMarginal_ = boost::none;
    }

  }

}
