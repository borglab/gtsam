/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DSF.h
 * @date Mar 26, 2010
 * @author Kai Ni
 * @brief An implementation of Disjoint set forests (see CLR page 446 and up)
 */

#pragma once

#include <gtsam_unstable/base/BTree.h>
#include <iostream>
#include <list>
#include <set>
#include <map>

namespace gtsam {

/**
 * Disjoint Set Forest class
 *
 * Quoting from CLR: A disjoint-set data structure maintains a collection
 * S = {S_1,S_2,...} of disjoint dynamic sets. Each set is identified by
 * a representative, which is some member of the set.
 *
 * @addtogroup base
 */
template<class KEY>
class DSF: protected BTree<KEY, KEY> {

public:
  typedef DSF<KEY> Self;
  typedef std::set<KEY> Set;
  typedef BTree<KEY, KEY> Tree;
  typedef std::pair<KEY, KEY> KeyLabel;

  // constructor
  DSF() :
      Tree() {
  }

  // constructor
  DSF(const Tree& tree) :
      Tree(tree) {
  }

  // constructor with a list of unconnected keys
  DSF(const std::list<KEY>& keys) :
      Tree() {
    for(const KEY& key: keys)
      *this = this->add(key, key);
  }

  // constructor with a set of unconnected keys
  DSF(const std::set<KEY>& keys) :
      Tree() {
    for(const KEY& key: keys)
      *this = this->add(key, key);
  }

  // create a new singleton, does nothing if already exists
  Self makeSet(const KEY& key) const {
    if (this->mem(key))
      return *this;
    else
      return this->add(key, key);
  }

  // create a new singleton, does nothing if already exists
  void makeSetInPlace(const KEY& key) {
    if (!this->mem(key))
      *this = this->add(key, key);
  }

  // find the label of the set in which {key} lives
  KEY findSet(const KEY& key) const {
    KEY parent = this->find(key);
    return parent == key ? key : findSet(parent);
  }

  // return a new DSF where x and y are in the same set. No path compression
  Self makeUnion(const KEY& key1, const KEY& key2) const {
    DSF<KEY> copy = *this;
    copy.makeUnionInPlace(key1,key2);
    return copy;
  }

  // the in-place version of makeUnion
  void makeUnionInPlace(const KEY& key1, const KEY& key2) {
    *this = this->add(findSet_(key2), findSet_(key1));
  }

  // create a new singleton with two connected keys
  Self makePair(const KEY& key1, const KEY& key2) const {
    return makeSet(key1).makeSet(key2).makeUnion(key1, key2);
  }

  // create a new singleton with a list of fully connected keys
  Self makeList(const std::list<KEY>& keys) const {
    Self t = *this;
    for(const KEY& key: keys)
      t = t.makePair(key, keys.front());
    return t;
  }

  // return a dsf in which all find_set operations will be O(1) due to path compression.
  DSF flatten() const {
    DSF t = *this;
    for(const KeyLabel& pair: (Tree)t)
      t.findSet_(pair.first);
    return t;
  }

  // maps f over all keys, must be invertible
  DSF map(std::function<KEY(const KEY&)> func) const {
    DSF t;
    for(const KeyLabel& pair: (Tree)*this)
      t = t.add(func(pair.first), func(pair.second));
    return t;
  }

  // return the number of sets
  size_t numSets() const {
    size_t num = 0;
    for(const KeyLabel& pair: (Tree)*this)
      if (pair.first == pair.second)
        num++;
    return num;
  }

  // return the numer of keys
  size_t size() const {
    return Tree::size();
  }

  // return all sets, i.e. a partition of all elements
  std::map<KEY, Set> sets() const {
    std::map<KEY, Set> sets;
    for(const KeyLabel& pair: (Tree)*this)
      sets[findSet(pair.second)].insert(pair.first);
    return sets;
  }

  // return a partition of the given elements {keys}
  std::map<KEY, Set> partition(const std::list<KEY>& keys) const {
    std::map<KEY, Set> partitions;
    for(const KEY& key: keys)
      partitions[findSet(key)].insert(key);
    return partitions;
  }

  // get the nodes in the tree with the given label
  Set set(const KEY& label) const {
    Set set;
    for(const KeyLabel& pair: (Tree)*this) {
      if (pair.second == label || findSet(pair.second) == label)
        set.insert(pair.first);
    }
    return set;
  }

  /** equality */
  bool operator==(const Self& t) const {
    return (Tree) *this == (Tree) t;
  }

  /** inequality */
  bool operator!=(const Self& t) const {
    return (Tree) *this != (Tree) t;
  }

  // print the object
  void print(const std::string& name = "DSF") const {
    std::cout << name << std::endl;
    for(const KeyLabel& pair: (Tree)*this)
      std::cout << (std::string) pair.first << " " << (std::string) pair.second
          << std::endl;
  }

protected:

  /**
   * same as findSet except with path compression: After we have traversed the path to
   * the root, each parent pointer is made to directly point to it
   */
  KEY findSet_(const KEY& key) {
    KEY parent = this->find(key);
    if (parent == key)
      return parent;
    else {
      KEY label = findSet_(parent);
      *this = this->add(key, label);
      return label;
    }
  }

};

// shortcuts
typedef DSF<int> DSFInt;

} // namespace gtsam
