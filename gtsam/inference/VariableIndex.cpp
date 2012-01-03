/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VariableIndex.cpp
 * @author  Richard Roberts
 * @date    Oct 22, 2010
 */

#include <iostream>

#include <gtsam/inference/VariableIndex.h>

namespace gtsam {

/* ************************************************************************* */
void VariableIndex::permute(const Permutation& permutation) {
#ifndef NDEBUG
  // Assert that the permutation does not leave behind any non-empty variables,
  // otherwise the nFactors and nEntries counts would be incorrect.
  for(Index j=0; j<this->index_.size(); ++j)
    if(find(permutation.begin(), permutation.end(), j) == permutation.end())
      assert(this->operator[](j).empty());
#endif
  index_.permute(permutation);
}

/* ************************************************************************* */
bool VariableIndex::equals(const VariableIndex& other, double tol) const {
  if(this->nEntries_ == other.nEntries_ && this->nFactors_ == other.nFactors_) {
    for(size_t var=0; var < std::max(this->index_.size(), other.index_.size()); ++var)
      if(var >= this->index_.size() || var >= other.index_.size()) {
        if(!((var >= this->index_.size() && other.index_[var].empty()) ||
            (var >= other.index_.size() && this->index_[var].empty())))
          return false;
      } else if(this->index_[var] != other.index_[var])
        return false;
  } else
    return false;
  return true;
}

/* ************************************************************************* */
void VariableIndex::print(const std::string& str) const {
  std::cout << str << "\n";
  std::cout << "nEntries = " << this->nEntries_ << ", nFactors = " << this->nFactors_ << "\n";
  Index var = 0;
  BOOST_FOREACH(const Factors& variable, index_.container()) {
    Permutation::const_iterator rvar = find(index_.permutation().begin(), index_.permutation().end(), var);
    assert(rvar != index_.permutation().end());
    std::cout << "var " << (rvar-index_.permutation().begin()) << ":";
    BOOST_FOREACH(const size_t factor, variable) {
      std::cout << " " << factor;
    }
    std::cout << "\n";
    ++ var;
  }
  std::cout << std::flush;
}

}
