/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VariableIndex-inl.h
 * @author  Richard Roberts
 * @date    March 26, 2013
 */

#pragma once

#include <gtsam/inference/VariableIndexUnordered.h>

namespace gtsam {

/* ************************************************************************* */
template<class FG>
void VariableIndexUnordered::augment(const FG& factors)
{
  gttic(VariableIndex_augment);

  // Save original number of factors for keeping track of indices
  const size_t originalNFactors = nFactors_;

  // Augment index for each factor
  for(size_t i = 0; i < factors.size(); ++i) {
    if(factors[i]) {
      const size_t globalI = originalNFactors + i;
      BOOST_FOREACH(const Key key, *factors[i]) {
        index_[key].push_back(globalI);
        ++ nEntries_;
      }
    }
    ++ nFactors_; // Increment factor count even if factors are null, to keep indices consistent
  }
}

/* ************************************************************************* */
template<typename ITERATOR, class FG>
void VariableIndexUnordered::remove(ITERATOR firstFactor, ITERATOR lastFactor, const FG& factors)
{
  gttic(VariableIndex_remove);

  // NOTE: We intentionally do not decrement nFactors_ because the factor
  // indices need to remain consistent.  Removing factors from a factor graph
  // does not shift the indices of other factors.  Also, we keep nFactors_
  // one greater than the highest-numbered factor referenced in a VariableIndex.
  ITERATOR factorIndex = firstFactor;
  size_t i = 0;
  for( ; factorIndex != lastFactor; ++factorIndex, ++i) {
    if(i >= factors.size())
      throw std::invalid_argument("Internal error, requested inconsistent number of factor indices and factors in VariableIndex::remove");
    if(factors[i]) {
      BOOST_FOREACH(Key j, *factors[i]) {
        Factors& factorEntries = internalAt(j);
        Factors::iterator entry = std::find(factorEntries.begin(), factorEntries.end(), *factorIndex);
        if(entry == factorEntries.end())
          throw std::invalid_argument("Internal error, indices and factors passed into VariableIndex::remove are not consistent with the existing variable index");
        factorEntries.erase(entry);
        -- nEntries_;
      }
    }
  }
}

/* ************************************************************************* */
template<typename ITERATOR>
void VariableIndexUnordered::removeUnusedVariables(ITERATOR firstKey, ITERATOR lastKey) {
  for(ITERATOR key = firstKey; key != lastKey; ++key) {
    assert(internalAt(*key).empty());
    index_.erase(*key);
  }
}

}
