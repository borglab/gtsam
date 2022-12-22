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
 * @brief A faster implementation for DSF, which uses vector rather than btree.
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/global_includes.h>

#include <boost/shared_ptr.hpp>

#include <vector>
#include <set>
#include <map>

namespace gtsam {

/**
 * A fast implementation of disjoint set forests that uses vector as underly data structure.
 * This is the absolute minimal DSF data structure, and only allows size_t keys
 * Uses rank compression but not union by rank :-(
 * @ingroup base
 */
class GTSAM_EXPORT DSFBase {

public:
  typedef std::vector<size_t> V; ///< Vector of ints

private:
  boost::shared_ptr<V> v_;///< Stores parent pointers, representative iff v[i]==i

public:
  /// Constructor that allocates new memory, allows for keys 0...numNodes-1.
  DSFBase(const size_t numNodes);

  /// Constructor that uses an existing, pre-allocated vector.
  DSFBase(const boost::shared_ptr<V>& v_in);

  /// Find the label of the set in which {key} lives.
  size_t find(size_t key) const;

  /// Merge the sets containing i1 and i2. Does nothing if i1 and i2 are already in the same set.
  void merge(const size_t& i1, const size_t& i2);
};

/**
 * DSFVector additionally keeps a vector of keys to support more expensive operations
 * @ingroup base
 */
class GTSAM_EXPORT DSFVector: public DSFBase {

private:
  std::vector<size_t> keys_; ///< stores keys to support more expensive operations

public:
  /// Constructor that allocates new memory, uses sequential keys 0...numNodes-1.
  DSFVector(const size_t numNodes);

  /// Constructor that allocates memory, uses given keys.
  DSFVector(const std::vector<size_t>& keys);

  /// Constructor that uses existing vectors.
  DSFVector(const boost::shared_ptr<V>& v_in, const std::vector<size_t>& keys);

  // All operations below loop over all keys and hence are *at least* O(n)

  /// Find whether there is one and only one occurrence for the given {label}.
  bool isSingleton(const size_t& label) const;

  /// Get the nodes in the tree with the given label
  std::set<size_t> set(const size_t& label) const;

  /// Return all sets, i.e. a partition of all elements.
  std::map<size_t, std::set<size_t> > sets() const;

  /// Return all sets, i.e. a partition of all elements.
  std::map<size_t, std::vector<size_t> > arrays() const;
};

}
