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

using namespace std;

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
  cout << str << "\n";
  cout << "nEntries = " << nEntries() << ", nFactors = " << nFactors() << "\n";
  Index var = 0;
  BOOST_FOREACH(const Factors& variable, index_.container()) {
    Permutation::const_iterator rvar = find(index_.permutation().begin(), index_.permutation().end(), var);
    assert(rvar != index_.permutation().end());
    cout << "var " << (rvar-index_.permutation().begin()) << ":";
    BOOST_FOREACH(const size_t factor, variable)
      cout << " " << factor;
    cout << "\n";
    ++ var;
  }
  cout << flush;
}

/* ************************************************************************* */
void VariableIndex::outputMetisFormat(ostream& os) const {
  os << size() << " " << nFactors() << "\n";
  // run over variables, which will be hyper-edges.
  BOOST_FOREACH(const Factors& variable, index_.container()) {
  	// every variable is a hyper-edge covering its factors
    BOOST_FOREACH(const size_t factor, variable)
      os << (factor+1) << " "; // base 1
    os << "\n";
  }
  os << flush;
}

}
