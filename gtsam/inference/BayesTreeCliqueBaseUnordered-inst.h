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

#include <gtsam/inference/BayesTreeCliqueBaseUnordered.h>
#include <gtsam/base/timing.h>
#include <boost/foreach.hpp>

namespace gtsam {

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  bool BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::equals(
    const DERIVED& other, double tol = 1e-9) const
  {
    return (!conditional_ && !other.conditional())
      || conditional_->equals(*other.conditional(), tol);
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  std::vector<Key>
    BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::separator_setminus_B(const derived_ptr& B) const
  {
    FastSet<Key> p_F_S_parents(this->conditional()->beginParents(), this->conditional()->endParents());
    FastSet<Key> indicesB(B->conditional()->begin(), B->conditional()->end());
    std::vector<Key> S_setminus_B;
    std::set_difference(p_F_S_parents.begin(), p_F_S_parents.end(),
      indicesB.begin(), indicesB.end(), back_inserter(S_setminus_B));
    return S_setminus_B;
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  std::vector<Key> BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::shortcut_indices(
    const derived_ptr& B, const FactorGraphType& p_Cp_B) const
  {
    gttic(shortcut_indices);
    FastSet<Key> allKeys = p_Cp_B.keys();
    FastSet<Key> indicesB(B->conditional()->begin(), B->conditional()->end());
    std::vector<Key> S_setminus_B = separator_setminus_B(B);
    std::vector<Key> keep;
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
  void BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::print(
    const std::string& s, const KeyFormatter& keyFormatter) const
  {
    conditional_->print(s, keyFormatter);
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  size_t BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::treeSize() const {
    size_t size = 1;
    BOOST_FOREACH(const derived_ptr& child, children)
      size += child->treeSize();
    return size;
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  size_t BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::numCachedSeparatorMarginals() const
  {
    if (!cachedSeparatorMarginal_)
      return 0;

    size_t subtree_count = 1;
    BOOST_FOREACH(const derived_ptr& child, children)
      subtree_count += child->numCachedSeparatorMarginals();

    return subtree_count;
  }

  /* ************************************************************************* */
  // The shortcut density is a conditional P(S|R) of the separator of this
  // clique on the root. We can compute it recursively from the parent shortcut
  // P(Sp|R) as \int P(Fp|Sp) P(Sp|R), where Fp are the frontal nodes in p
  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  typename BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::BayesNetType
    BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::shortcut(const derived_ptr& B, Eliminate function) const
  {
    gttic(BayesTreeCliqueBaseUnordered_shortcut);
    // We only calculate the shortcut when this clique is not B
    // and when the S\B is not empty
    std::vector<Key> S_setminus_B = separator_setminus_B(B);
    if (!parent_.expired() /*(if we're not the root)*/ && !S_setminus_B.empty())
    {
      // Obtain P(Cp||B) = P(Fp|Sp) * P(Sp||B) as a factor graph
      derived_ptr parent(parent_.lock());
      gttoc(BayesTreeCliqueBaseUnordered_shortcut);
      FactorGraphType p_Cp_B(parent->shortcut(B, function)); // P(Sp||B)
      gttic(BayesTreeCliqueBaseUnordered_shortcut);
      p_Cp_B += parent->conditional_; // P(Fp|Sp)

      // Determine the variables we want to keepSet, S union B
      std::vector<Key> keep = shortcut_indices(B, p_Cp_B);

      // Marginalize out everything except S union B
      BayesNetType result = *p_Cp_B.marginalMultifrontalBayesNet(
        OrderingUnordered(keep), boost::none, function);

      // Finally, we only want to have S\B variables in the Bayes net, so
      size_t nrFrontals = S_setminus_B.size();
      result.erase(result.begin() + nrFrontals, result.end());

      return result;
    }
    else
    {
      return BayesNetType();
    }
  }

  /* ************************************************************************* */
  // separator marginal, uses separator marginal of parent recursively
  // P(C) = P(F|S) P(S)
  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  typename BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::FactorGraphType
    BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::separatorMarginal(Eliminate function) const
  {
    gttic(BayesTreeCliqueBaseUnordered_separatorMarginal);
    // Check if the Separator marginal was already calculated
    if (!cachedSeparatorMarginal_)
    {
      gttic(BayesTreeCliqueBaseUnordered_separatorMarginal_cachemiss);
      // If this is the root, there is no separator
      if (parent_.expired() /*(if we're the root)*/)
      {
        // we are root, return empty
        FactorGraphType empty;
        cachedSeparatorMarginal_ = empty;
      }
      else
      {
        // Obtain P(S) = \int P(Cp) = \int P(Fp|Sp) P(Sp)
        // initialize P(Cp) with the parent separator marginal
        derived_ptr parent(parent_.lock());
        gttoc(BayesTreeCliqueBaseUnordered_separatorMarginal_cachemiss); // Flatten recursion in timing outline
        gttoc(BayesTreeCliqueBaseUnordered_separatorMarginal);
        FactorGraphType p_Cp(parent->separatorMarginal(function)); // P(Sp)
        gttic(BayesTreeCliqueBaseUnordered_separatorMarginal);
        gttic(BayesTreeCliqueBaseUnordered_separatorMarginal_cachemiss);
        // now add the parent conditional
        p_Cp += parent->conditional_; // P(Fp|Sp)

        // The variables we want to keepSet are exactly the ones in S
        std::vector<Key> indicesS(this->conditional()->beginParents(), this->conditional()->endParents());
        cachedSeparatorMarginal_ = *p_Cp.marginalMultifrontalBayesNet(OrderingUnordered(indicesS), boost::none, function);
      }
    }

    // return the shortcut P(S||B)
    return *cachedSeparatorMarginal_; // return the cached version
  }

  /* ************************************************************************* */
  // marginal2, uses separator marginal of parent recursively
  // P(C) = P(F|S) P(S)
  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  typename BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::FactorGraphType
    BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::marginal2(Eliminate function) const
  {
    gttic(BayesTreeCliqueBaseUnordered_marginal2);
    // initialize with separator marginal P(S)
    FactorGraphType p_C = this->separatorMarginal(function);
    // add the conditional P(F|S)
    p_C += boost::shared_ptr<FactorType>(this->conditional_);
    return p_C;
  }

  /* ************************************************************************* */
  template<class DERIVED, class FACTORGRAPH>
  void BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH>::deleteCachedShortcuts() {

    // When a shortcut is requested, all of the shortcuts between it and the
    // root are also generated. So, if this clique's cached shortcut is set,
    // recursively call over all child cliques. Otherwise, it is unnecessary.
    if (cachedSeparatorMarginal_) {
      BOOST_FOREACH(derived_ptr& child, children) {
        child->deleteCachedShortcuts();
      }

      //Delete CachedShortcut for this clique
      cachedSeparatorMarginal_ = boost::none;
    }

  }

}
