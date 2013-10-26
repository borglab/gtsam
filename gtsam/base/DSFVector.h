/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DSFVector.h
 * @date Jun 25, 2010
 * @author Kai Ni
 * @brief A faster implementation for DSF, which uses vector rather than btree. As a result, the size of the forest is prefixed.
 */

#pragma once

#include <gtsam/global_includes.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <set>
#include <map>

namespace gtsam {

/**
 * A fast implementation of disjoint set forests that uses vector as underly data structure.
 * @addtogroup base
 */
class GTSAM_EXPORT DSFVector {

public:
  typedef std::vector<size_t> V; ///< Vector of ints

private:
  boost::shared_ptr<V> v_;///< Stores parent pointers, representative iff v[i]==i
  std::vector<size_t> keys_;///< stores keys

public:
  /// constructor that allocate new memory, uses sequential keys 0...numNodes-1
  DSFVector(const size_t numNodes);

  /// constructor that allocates memory, uses given keys
  DSFVector(const std::vector<size_t>& keys);

  /// constructor that uses the existing memory
  DSFVector(const boost::shared_ptr<V>& v_in, const std::vector<size_t>& keys);

  /// find the label of the set in which {key} lives
  inline size_t findSet(size_t key) const {
    size_t parent = (*v_)[key];
    // follow parent pointers until we reach set representative
    while (parent != key) {
      key = parent;
      parent = (*v_)[key];
    }
    return parent;
  }

  /// find whether there is one and only one occurrence for the given {label}
  bool isSingleton(const size_t& label) const;

  /// get the nodes in the tree with the given label
  std::set<size_t> set(const size_t& label) const;

  /// return all sets, i.e. a partition of all elements
  std::map<size_t, std::set<size_t> > sets() const;

  /// return all sets, i.e. a partition of all elements
  std::map<size_t, std::vector<size_t> > arrays() const;

  /// the in-place version of makeUnion
  void makeUnionInPlace(const size_t& i1, const size_t& i2);
};

}
