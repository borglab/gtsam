/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Permutation.cpp
 * @author  Richard Roberts
 * @date    Oct 9, 2010
 */

#include <gtsam/inference/Permutation.h>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <boost/foreach.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Permutation Permutation::Identity(Index nVars) {
  Permutation ret(nVars);
  for(Index i=0; i<nVars; ++i)
    ret[i] = i;
  return ret;
}

/* ************************************************************************* */
Permutation Permutation::PullToFront(const vector<Index>& toFront, size_t size, bool filterDuplicates) {

  Permutation ret(size);

  // Mask of which variables have been pulled, used to reorder
  vector<bool> pulled(size, false);

  // Put the pulled variables at the front of the permutation and set up the
  // pulled flags.
  size_t toFrontUniqueSize = 0;
  for(Index j=0; j<toFront.size(); ++j) {
    if(!pulled[toFront[j]]) {
      ret[j] = toFront[j];
      pulled[toFront[j]] = true;
      ++ toFrontUniqueSize;
    } else if(!filterDuplicates) {
      stringstream ss;
      ss << "Duplicate variable given as input to Permutation::PullToFront:\n";
      ss << "    toFront:";
      BOOST_FOREACH(Index i, toFront) { ss << " " << i; }
      ss << ", size = " << size << endl;
      throw invalid_argument(ss.str());
    }
  }

  // Fill in the rest of the variables
  Index nextVar = toFrontUniqueSize;
  for(Index j=0; j<size; ++j)
    if(!pulled[j])
      ret[nextVar++] = j;
  assert(nextVar == size);

  return ret;
}

/* ************************************************************************* */
Permutation Permutation::PushToBack(const std::vector<Index>& toBack, size_t size, bool filterDuplicates) {

  Permutation ret(size);

  // Mask of which variables have been pushed, used to reorder
  vector<bool> pushed(size, false);

  // Put the pushed variables at the back of the permutation and set up the
  // pushed flags;
  Index nextVar = size;
  size_t toBackUniqueSize = 0;
  if(toBack.size() > 0) {
    Index j = toBack.size();
    do {
      -- j;
      if(!pushed[toBack[j]]) {
        ret[--nextVar] = toBack[j];
        pushed[toBack[j]] = true;
        ++ toBackUniqueSize;
      } else if(!filterDuplicates) {
        stringstream ss;
        ss << "Duplicate variable given as input to Permutation::PushToBack:\n";
        ss << "    toBack:";
        BOOST_FOREACH(Index i, toBack) { ss << " " << i; }
        ss << ", size = " << size << endl;
        throw invalid_argument(ss.str());
      }
    } while(j > 0);
  }
  assert(nextVar == size - toBackUniqueSize);

  // Fill in the rest of the variables
  nextVar = 0;
  for(Index j=0; j<size; ++j)
    if(!pushed[j])
      ret[nextVar++] = j;
  assert(nextVar == size - toBackUniqueSize);

  return ret;
}

/* ************************************************************************* */
Permutation::shared_ptr Permutation::permute(const Permutation& permutation) const {
  const size_t nVars = permutation.size();
  Permutation::shared_ptr result(new Permutation(nVars));
  for(size_t j=0; j<nVars; ++j) {
    assert(permutation[j] < rangeIndices_.size());
    (*result)[j] = operator[](permutation[j]);
  }
  return result;
}

/* ************************************************************************* */
Permutation::shared_ptr Permutation::partialPermutation(
    const Permutation& selector, const Permutation& partialPermutation) const {
  assert(selector.size() == partialPermutation.size());
  Permutation::shared_ptr result(new Permutation(*this));

  for(Index subsetPos=0; subsetPos<selector.size(); ++subsetPos)
    (*result)[selector[subsetPos]] = (*this)[selector[partialPermutation[subsetPos]]];

  return result;
}

/* ************************************************************************* */
Permutation::shared_ptr Permutation::inverse() const {
  Permutation::shared_ptr result(new Permutation(this->size()));
  for(Index i=0; i<this->size(); ++i) {
    assert((*this)[i] < this->size());
    (*result)[(*this)[i]] = i;
  }
  return result;
}

/* ************************************************************************* */
void Permutation::print(const std::string& str) const {
  std::cout << str << " ";
  BOOST_FOREACH(Index s, rangeIndices_) { std::cout << s << " "; }
  std::cout << std::endl;
}

}
