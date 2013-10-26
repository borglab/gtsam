/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DSFMap.h
 * @date Oct 26, 2013
 * @author Frank Dellaert
 * @brief Allow for arbitrary type in DSF
 */

#pragma once

#include <map>
#include <set>
#include <boost/foreach.hpp>


namespace gtsam {

/**
 * Disjoint set forest using an STL map data structure underneath
 * Uses rank compression but not union by rank :-(
 * @addtogroup base
 */
template<class KEY>
class DSFMap {

  /// We store the forest in an STL map
  typedef std::map<KEY, KEY> Map;
  typedef std::set<KEY> Set;
  typedef std::pair<KEY, KEY> key_pair;
  mutable Map parent_;

public:
  /// constructor
  DSFMap() {}

  /// find the label of the set in which {key} lives
  KEY find(const KEY& key) const {
    typename Map::const_iterator it = parent_.find(key);
    // if key does not exist, create and return itself
    if (it==parent_.end()) {
      parent_[key] = key;
      return key;
    } else {
      // follow parent pointers until we reach set representative
      KEY parent = it->second;
      if (parent != key)
        parent = find(parent); // not yet, recurse!
      parent_[key] = parent; // path compression
      return parent;
    }
  }

  /// Merge two sets
  void merge(const KEY& i1, const KEY& i2) {
    parent_[find(i2)] = find(i1);
  }

  /// return all sets, i.e. a partition of all elements
  std::map<KEY, Set> sets() const {
    std::map<KEY, Set> sets;
    BOOST_FOREACH(const key_pair& pair, parent_)
      sets[find(pair.second)].insert(pair.first);
    return sets;
  }

};

}
