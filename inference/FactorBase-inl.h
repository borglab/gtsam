/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Factor-inl.h
 * @brief   
 * @author  Richard Roberts
 * @created Sep 1, 2010
 */

#pragma once

#include <gtsam/inference/FactorBase.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <iostream>

namespace gtsam {

/* ************************************************************************* */
template<typename KEY>
FactorBase<KEY>::FactorBase(const FactorBase<KEY>& f) : keys_(f.keys_) {}

/* ************************************************************************* */
template<typename KEY>
FactorBase<KEY>::FactorBase(const Conditional& c) : keys_(c.keys()) {}

/* ************************************************************************* */
template<typename KEY>
void FactorBase<KEY>::assertInvariants() const {
#ifndef NDEBUG
  std::set<Index> uniqueSorted(keys_.begin(), keys_.end());
  assert(uniqueSorted.size() == keys_.size());
  assert(std::equal(uniqueSorted.begin(), uniqueSorted.end(), keys_.begin()));
#endif
}

/* ************************************************************************* */
template<typename KEY>
void FactorBase<KEY>::print(const std::string& s) const {
  std::cout << s << " ";
  BOOST_FOREACH(KEY key, keys_) std::cout << " " << key;
  std::cout << std::endl;
}

/* ************************************************************************* */
template<typename KEY>
//template<class DERIVED>
bool FactorBase<KEY>::equals(const This& other, double tol) const {
  return keys_ == other.keys_;
}

/* ************************************************************************* */
template<typename KEY>
template<class DERIVED>
typename DERIVED::shared_ptr FactorBase<KEY>::Combine(const FactorGraph<DERIVED>& factors, const FastMap<Key, std::vector<Key> >& variableSlots) {
  typedef const FastMap<Key, std::vector<Key> > VariableSlots;
  typedef typeof(boost::lambda::bind(&VariableSlots::value_type::first, boost::lambda::_1)) FirstGetter;
  typedef boost::transform_iterator<
      FirstGetter, typename VariableSlots::const_iterator,
      KEY, KEY> IndexIterator;
  FirstGetter firstGetter(boost::lambda::bind(&VariableSlots::value_type::first, boost::lambda::_1));
  IndexIterator keysBegin(variableSlots.begin(), firstGetter);
  IndexIterator keysEnd(variableSlots.end(), firstGetter);
  return typename DERIVED::shared_ptr(new DERIVED(keysBegin, keysEnd));
}

/* ************************************************************************* */
template<typename KEY>
template<class CONDITIONAL>
typename CONDITIONAL::shared_ptr FactorBase<KEY>::eliminateFirst() {
  assert(!keys_.empty());
  assertInvariants();
  KEY eliminated = keys_.front();
  keys_.erase(keys_.begin());
  return typename CONDITIONAL::shared_ptr(new CONDITIONAL(eliminated, keys_));
}

/* ************************************************************************* */
template<typename KEY>
template<class CONDITIONAL>
typename BayesNet<CONDITIONAL>::shared_ptr FactorBase<KEY>::eliminate(size_t nrFrontals) {
  assert(keys_.size() >= nrFrontals);
  assertInvariants();
  typename BayesNet<CONDITIONAL>::shared_ptr fragment(new BayesNet<CONDITIONAL>());
  const_iterator nextFrontal = this->begin();
  for(KEY n = 0; n < nrFrontals; ++n, ++nextFrontal)
    fragment->push_back(CONDITIONAL::FromRange(
        nextFrontal, const_iterator(this->end()), 1));
  if(nrFrontals > 0)
    keys_.assign(fragment->back()->beginParents(), fragment->back()->endParents());
  return fragment;
}

/* ************************************************************************* */
template<typename KEY>
void FactorBase<KEY>::permuteWithInverse(const Permutation& inversePermutation) {
  BOOST_FOREACH(KEY& key, keys_) { key = inversePermutation[key]; }
}

}
