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
 * @date    March 26, 2013
 */

#include <iostream>

#include <gtsam/inference/VariableIndex.h>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
bool VariableIndex::equals(const VariableIndex& other, double tol) const {
  return this->nEntries_ == other.nEntries_ && this->nFactors_ == other.nFactors_
    && this->index_ == other.index_;
}

/* ************************************************************************* */
void VariableIndex::print(const string& str, const KeyFormatter& keyFormatter) const {
  cout << str;
  cout << "nEntries = " << nEntries() << ", nFactors = " << nFactors() << "\n";
  for(KeyMap::value_type key_factors: index_) {
    cout << "var " << keyFormatter(key_factors.first) << ":";
    for(const auto index: key_factors.second)
      cout << " " << index;
    cout << "\n";
  }
  cout.flush();
}

/* ************************************************************************* */
void VariableIndex::outputMetisFormat(ostream& os) const {
  os << size() << " " << nFactors() << "\n";
  // run over variables, which will be hyper-edges.
  for(KeyMap::value_type key_factors: index_) {
    // every variable is a hyper-edge covering its factors
    for(const auto index: key_factors.second)
      os << (index+1) << " "; // base 1
    os << "\n";
  }
  os << flush;
}

/* ************************************************************************* */
void VariableIndex::augmentExistingFactor(const FactorIndex factorIndex, const KeySet & newKeys)
{
  gttic(VariableIndex_augmentExistingFactor);

  for(const Key key: newKeys) {
    index_[key].push_back(factorIndex);
    ++nEntries_;
  }

  gttoc(VariableIndex_augmentExistingFactor);
}

}
