/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DSFVector.cpp
 * @date Jun 25, 2010
 * @author Kai Ni
 * @brief a faster implementation for DSF, which uses vector rather than btree.
 */

#include <gtsam/base/DSFVector.h>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <algorithm>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
DSFBase::DSFBase(const size_t numNodes) {
  v_ = boost::make_shared < V > (numNodes);
  int index = 0;
  for (V::iterator it = v_->begin(); it != v_->end(); it++, index++)
    *it = index;
}

/* ************************************************************************* */
DSFBase::DSFBase(const boost::shared_ptr<V>& v_in) {
  v_ = v_in;
  int index = 0;
  for (V::iterator it = v_->begin(); it != v_->end(); it++, index++)
    *it = index;
}

/* ************************************************************************* */
size_t DSFBase::findSet(size_t key) const {
  size_t parent = (*v_)[key];
  // follow parent pointers until we reach set representative
  while (parent != key) {
    key = parent;
    parent = (*v_)[key];
  }
  return parent;
}

/* ************************************************************************* */
void DSFBase::makeUnionInPlace(const size_t& i1, const size_t& i2) {
  (*v_)[findSet(i2)] = findSet(i1);
}

/* ************************************************************************* */
DSFVector::DSFVector(const size_t numNodes) :
    DSFBase(numNodes) {
  keys_.reserve(numNodes);
  for (size_t index = 0; index < numNodes; index++)
    keys_.push_back(index);
}

/* ************************************************************************* */
DSFVector::DSFVector(const std::vector<size_t>& keys) :
    DSFBase(1 + *std::max_element(keys.begin(), keys.end())), keys_(keys) {
}

/* ************************************************************************* */
DSFVector::DSFVector(const boost::shared_ptr<V>& v_in,
    const std::vector<size_t>& keys) :
    DSFBase(v_in), keys_(keys) {
  assert(*(std::max_element(keys.begin(), keys.end()))<v_in->size());
}

/* ************************************************************************* */
bool DSFVector::isSingleton(const size_t& label) const {
  bool result = false;
  BOOST_FOREACH(size_t key,keys_) {
    if (findSet(key) == label) {
      if (!result) // find the first occurrence
        result = true;
      else
        return false;
    }
  }
  return result;
}

/* ************************************************************************* */
std::set<size_t> DSFVector::set(const size_t& label) const {
  std::set < size_t > set;
  BOOST_FOREACH(size_t key,keys_)
    if (findSet(key) == label)
      set.insert(key);
  return set;
}

/* ************************************************************************* */
std::map<size_t, std::set<size_t> > DSFVector::sets() const {
  std::map<size_t, std::set<size_t> > sets;
  BOOST_FOREACH(size_t key,keys_)
    sets[findSet(key)].insert(key);
  return sets;
}

/* ************************************************************************* */
std::map<size_t, std::vector<size_t> > DSFVector::arrays() const {
  std::map<size_t, std::vector<size_t> > arrays;
  BOOST_FOREACH(size_t key,keys_)
    arrays[findSet(key)].push_back(key);
  return arrays;
}

} // namespace  gtsam

