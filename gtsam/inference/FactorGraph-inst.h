/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   FactorGraph-inl.h
 * @brief  Factor Graph Base Class
 * @author Carlos Nieto
 * @author Frank Dellaert
 * @author Alireza Fathi
 * @author Michael Kaess
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/inference/FactorGraph.h>

#include <stdio.h>
#include <algorithm>
#include <iostream>  // for cout :-(
#include <sstream>
#include <string>

namespace gtsam {

/* ************************************************************************* */
template <class FACTOR>
void FactorGraph<FACTOR>::print(const std::string& s,
                                const KeyFormatter& formatter) const {
  std::cout << (s.empty() ? "" : s + " ") << std::endl;
  std::cout << "size: " << size() << std::endl;
  for (size_t i = 0; i < factors_.size(); i++) {
    std::stringstream ss;
    ss << "factor " << i << ": ";
    if (factors_[i]) factors_[i]->print(ss.str(), formatter);
  }
}

/* ************************************************************************* */
template <class FACTOR>
bool FactorGraph<FACTOR>::equals(const This& fg, double tol) const {
  // check whether the two factor graphs have the same number of factors.
  if (factors_.size() != fg.size()) return false;

  // check whether the factors are the same, in same order.
  for (size_t i = 0; i < factors_.size(); i++) {
    sharedFactor f1 = factors_[i], f2 = fg.factors_[i];
    if (f1 == nullptr && f2 == nullptr) continue;
    if (f1 == nullptr || f2 == nullptr) return false;
    if (!f1->equals(*f2, tol)) return false;
  }
  return true;
}

/* ************************************************************************* */
template <class FACTOR>
size_t FactorGraph<FACTOR>::nrFactors() const {
  size_t size_ = 0;
  for (const sharedFactor& factor : factors_)
    if (factor) size_++;
  return size_;
}

/* ************************************************************************* */
template <class FACTOR>
KeySet FactorGraph<FACTOR>::keys() const {
  KeySet keys;
  for (const sharedFactor& factor : this->factors_) {
    if (factor) keys.insert(factor->begin(), factor->end());
  }
  return keys;
}

/* ************************************************************************* */
template <class FACTOR>
KeyVector FactorGraph<FACTOR>::keyVector() const {
  KeyVector keys;
  keys.reserve(2 * size());  // guess at size
  for (const sharedFactor& factor : factors_)
    if (factor) keys.insert(keys.end(), factor->begin(), factor->end());
  std::sort(keys.begin(), keys.end());
  auto last = std::unique(keys.begin(), keys.end());
  keys.erase(last, keys.end());
  return keys;
}

/* ************************************************************************* */
template <class FACTOR>
template <typename CONTAINER, typename>
FactorIndices FactorGraph<FACTOR>::add_factors(const CONTAINER& factors,
                                               bool useEmptySlots) {
  const size_t num_factors = factors.size();
  FactorIndices newFactorIndices(num_factors);
  if (useEmptySlots) {
    size_t i = 0;
    for (size_t j = 0; j < num_factors; ++j) {
      // Loop to find the next available factor slot
      do {
        if (i >= size())
          // Make room for remaining factors, happens only once.
          resize(size() + num_factors - j);
        else if (at(i))
          ++i;  // Move on to the next slot or past end.
        else
          break;  // We found an empty slot, break to fill it.
      } while (true);

      // Use the current slot, updating graph and newFactorSlots.
      at(i) = factors[j];
      newFactorIndices[j] = i;
    }
  } else {
    // We're not looking for unused slots, so just add the factors at the end.
    for (size_t i = 0; i < num_factors; ++i) newFactorIndices[i] = i + size();
    push_back(factors);
  }
  return newFactorIndices;
}

}  // namespace gtsam
