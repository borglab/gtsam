/**
 * @file    Factor-inl.h
 * @brief   
 * @author  Richard Roberts
 * @created Sep 1, 2010
 */

#pragma once

#include <gtsam/inference/Factor.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

namespace gtsam {

///* ************************************************************************* */
//template<class ConditionalType>
//Factor::Factor(const boost::shared_ptr<ConditionalType>& c) {
//  keys_.resize(c->parents().size()+1);
//  keys_[0] = c->key();
//  size_t j = 1;
//  BOOST_FOREACH(const varid_t parent, c->parents()) {
//    keys_[j++] = parent;
//  }
//  checkSorted();
//}

/* ************************************************************************* */
template<class Derived> Factor::Factor(const Derived& c) : keys_(c.keys()), permuted_(c.permuted_) {
}

/* ************************************************************************* */
template<class KeyIterator> Factor::Factor(KeyIterator beginKey, KeyIterator endKey) :
    keys_(beginKey, endKey) { checkSorted(); }

/* ************************************************************************* */
template<class FactorGraphType, class VariableIndexStorage>
Factor::shared_ptr Factor::Combine(const FactorGraphType& factorGraph,
    const VariableIndex<VariableIndexStorage>& variableIndex, const std::vector<size_t>& factors,
    const std::vector<varid_t>& variables, const std::vector<std::vector<size_t> >& variablePositions) {

  return shared_ptr(boost::make_shared<Factor>(variables.begin(), variables.end()));
}

/* ************************************************************************* */
template<class MapAllocator>
Factor::shared_ptr Factor::Combine(const FactorGraph<Factor>& factors, const std::map<varid_t, std::vector<varid_t>, std::less<varid_t>, MapAllocator>& variableSlots) {
  typedef const std::map<varid_t, std::vector<varid_t>, std::less<varid_t>, MapAllocator> VariableSlots;
  typedef typeof(boost::lambda::bind(&VariableSlots::value_type::first, boost::lambda::_1)) FirstGetter;
  typedef boost::transform_iterator<
      FirstGetter, typename VariableSlots::const_iterator,
      varid_t, varid_t> IndexIterator;
  FirstGetter firstGetter(boost::lambda::bind(&VariableSlots::value_type::first, boost::lambda::_1));
  IndexIterator keysBegin(variableSlots.begin(), firstGetter);
  IndexIterator keysEnd(variableSlots.end(), firstGetter);
  return shared_ptr(new Factor(keysBegin, keysEnd));
}

}
