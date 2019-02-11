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
#include <algorithm>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
DSFBase::DSFBase(const size_t numNodes) {
  v_ = std::make_shared < V > (numNodes);
  int index = 0;
  for (V::iterator it = v_->begin(); it != v_->end(); it++, index++)
    *it = index;
}

/* ************************************************************************* */
DSFBase::DSFBase(const std::shared_ptr<V>& v_in) {
  v_ = v_in;
  int index = 0;
  for (V::iterator it = v_->begin(); it != v_->end(); it++, index++)
    *it = index;
}

/* ************************************************************************* */
size_t DSFBase::find(size_t key) const {
  // follow parent pointers until we reach set representative
  size_t parent = (*v_)[key];
  if (parent != key)
    parent = find(parent); // recursive call
  (*v_)[key] = parent; // path compression
  return parent;
}

/* ************************************************************************* */
void DSFBase::merge(const size_t& i1, const size_t& i2) {
  (*v_)[find(i2)] = find(i1);
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
DSFVector::DSFVector(const std::shared_ptr<V>& v_in,
    const std::vector<size_t>& keys) :
    DSFBase(v_in), keys_(keys) {
  assert(*(std::max_element(keys.begin(), keys.end()))<v_in->size());
}

/* ************************************************************************* */
bool DSFVector::isSingleton(const size_t& label) const {
  bool result = false;
  for(size_t key: keys_) {
    if (find(key) == label) {
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
  for(size_t key: keys_)
    if (find(key) == label)
      set.insert(key);
  return set;
}

/* ************************************************************************* */
std::map<size_t, std::set<size_t> > DSFVector::sets() const {
  std::map<size_t, std::set<size_t> > sets;
  for(size_t key: keys_)
    sets[find(key)].insert(key);
  return sets;
}

/* ************************************************************************* */
std::map<size_t, std::vector<size_t> > DSFVector::arrays() const {
  std::map<size_t, std::vector<size_t> > arrays;
  for(size_t key: keys_)
    arrays[find(key)].push_back(key);
  return arrays;
}

} // namespace  gtsam

