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
#include <gtsam/inference/Permutation.h>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
bool VariableIndex::equals(const VariableIndex& other, double tol) const {
  if(this->nEntries_ == other.nEntries_ && this->nFactors_ == other.nFactors_) {
    for(size_t var=0; var < max(this->index_.size(), other.index_.size()); ++var)
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
void VariableIndex::print(const string& str) const {
  cout << str;
  cout << "nEntries = " << nEntries() << ", nFactors = " << nFactors() << "\n";
  for(Index var = 0; var < size(); ++var) {
    cout << "var " << var << ":";
    BOOST_FOREACH(const size_t factor, index_[var])
      cout << " " << factor;
    cout << "\n";
  }
  cout << flush;
}

/* ************************************************************************* */
void VariableIndex::outputMetisFormat(ostream& os) const {
  os << size() << " " << nFactors() << "\n";
  // run over variables, which will be hyper-edges.
  BOOST_FOREACH(const Factors& variable, index_) {
    // every variable is a hyper-edge covering its factors
    BOOST_FOREACH(const size_t factor, variable)
      os << (factor+1) << " "; // base 1
    os << "\n";
  }
  os << flush;
}

/* ************************************************************************* */
void VariableIndex::permuteInPlace(const Permutation& permutation) {
  // Create new index and move references to data into it in permuted order
  vector<VariableIndex::Factors> newIndex(this->size());
  for(Index i = 0; i < newIndex.size(); ++i)
    newIndex[i].swap(this->index_[permutation[i]]);

  // Move reference to entire index into the VariableIndex
  index_.swap(newIndex);
}

/* ************************************************************************* */
void VariableIndex::permuteInPlace(const Permutation& selector, const Permutation& permutation) {
  if(selector.size() != permutation.size())
    throw invalid_argument("VariableIndex::permuteInPlace (partial permutation version) called with selector and permutation of different sizes.");
  // Create new index the size of the permuted entries
  vector<VariableIndex::Factors> newIndex(selector.size());
  // Permute the affected entries into the new index
  for(size_t dstSlot = 0; dstSlot < selector.size(); ++dstSlot)
    newIndex[dstSlot].swap(this->index_[selector[permutation[dstSlot]]]);
  // Put the affected entries back in the new order
  for(size_t slot = 0; slot < selector.size(); ++slot)
    this->index_[selector[slot]].swap(newIndex[slot]);
}

/* ************************************************************************* */
void VariableIndex::removeUnusedAtEnd(size_t nToRemove) {
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
  for(size_t i = this->size() - nToRemove; i < this->size(); ++i)
    if(!(*this)[i].empty())
      throw std::invalid_argument("Attempting to remove non-empty variables with VariableIndex::removeUnusedAtEnd()");
#endif

  index_.resize(this->size() - nToRemove);
}

}
