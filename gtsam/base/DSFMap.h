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

#include <cstdlib>  // Provides size_t
#include <map>
#include <set>
#include <vector>

namespace gtsam {

/**
 * Disjoint set forest using an STL map data structure underneath
 * Uses rank compression and union by rank, iterator version
 * @ingroup base
 */
template <class KEY>
class DSFMap {
 protected:
  /// We store the forest in an STL map, but parents are done with pointers
  struct Entry {
    typename std::map<KEY, Entry>::iterator parent_;
    size_t rank_;
    Entry() {}
  };

  typedef typename std::map<KEY, Entry> Map;
  typedef typename Map::iterator iterator;
  mutable Map entries_;

  /// Given key, find iterator to initial entry
  iterator find__(const KEY& key) const {
    static const Entry empty;
    iterator it = entries_.find(key);
    // if key does not exist, create and return itself
    if (it == entries_.end()) {
      it = entries_.insert({key, empty}).first;
      it->second.parent_ = it;
      it->second.rank_ = 0;
    }
    return it;
  }

  /// Given iterator to initial entry, find the root Entry
  iterator find_(const iterator& it) const {
    // follow parent pointers until we reach set representative
    iterator& parent = it->second.parent_;
    if (parent != it) parent = find_(parent);  // not yet, recurse!
    return parent;
  }

  /// Given key, find the root Entry
  inline iterator find_(const KEY& key) const {
    iterator initial = find__(key);
    return find_(initial);
  }

 public:
  typedef std::set<KEY> Set;

  /// constructor
  DSFMap() {}

  /// Given key, find the representative key for the set in which it lives
  inline KEY find(const KEY& key) const {
    iterator root = find_(key);
    return root->first;
  }

  /// Merge two sets
  void merge(const KEY& x, const KEY& y) {
    // straight from http://en.wikipedia.org/wiki/Disjoint-set_data_structure
    iterator xRoot = find_(x);
    iterator yRoot = find_(y);
    if (xRoot == yRoot) return;

    // Merge sets
    if (xRoot->second.rank_ < yRoot->second.rank_)
      xRoot->second.parent_ = yRoot;
    else if (xRoot->second.rank_ > yRoot->second.rank_)
      yRoot->second.parent_ = xRoot;
    else {
      yRoot->second.parent_ = xRoot;
      xRoot->second.rank_ = xRoot->second.rank_ + 1;
    }
  }

  /// return all sets, i.e. a partition of all elements
  std::map<KEY, Set> sets() const {
    std::map<KEY, Set> sets;
    iterator it = entries_.begin();
    for (; it != entries_.end(); it++) {
      iterator root = find_(it);
      sets[root->first].insert(it->first);
    }
    return sets;
  }
};

/// Small utility class for representing a wrappable pairs of ints.
class IndexPair : public std::pair<size_t,size_t> {
 public:
  inline IndexPair(): std::pair<size_t,size_t>(0,0) {}
  inline IndexPair(size_t i, size_t j) : std::pair<size_t,size_t>(i,j) {}
  inline size_t i() const { return first; };
  inline size_t j() const { return second; };
};

typedef std::vector<IndexPair> IndexPairVector;
typedef std::set<IndexPair> IndexPairSet;

inline IndexPairVector IndexPairSetAsArray(IndexPairSet& set) { return IndexPairVector(set.begin(), set.end()); }

typedef std::map<IndexPair, IndexPairSet> IndexPairSetMap;
typedef DSFMap<IndexPair> DSFMapIndexPair;
}  // namespace gtsam
