/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    MetisIndex.h
 * @author  Andrew Melim
 * @date    Oct. 10, 2014
 */

#pragma once


#include <gtsam/inference/Key.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/base/types.h>
#include <gtsam/base/timing.h>

#include <vector>
#include <map>
#include <unordered_map>

namespace gtsam {
/**
 * The MetisIndex class converts a factor graph into the Compressed Sparse Row format for use in
 * METIS algorithms. Specifically, two vectors store the adjacency structure of the graph. It is built
 * from a factor graph prior to elimination, and stores the list of factors
 * that involve each variable.
 * \nosubgrouping
 */
class GTSAM_EXPORT MetisIndex {
public:
  typedef std::shared_ptr<MetisIndex> shared_ptr;

private:
  // Stores Key <-> integer value relationship
 struct BiMap {
   std::map<Key, int32_t> left;
   std::unordered_map<int32_t, Key> right;
   void insert(const Key& left_value, const int32_t& right_value) {
     left[left_value] = right_value;
     right[right_value] = left_value;
   }
 };

  std::vector<int32_t> xadj_; // Index of node's adjacency list in adj
  std::vector<int32_t> adj_; // Stores ajacency lists of all nodes, appended into a single vector
  BiMap intKeyBMap_; // Stores Key <-> integer value relationship
  size_t nKeys_;

public:
  /// @name Standard Constructors
  /// @{

  /** Default constructor, creates empty MetisIndex */
  MetisIndex() :
      nKeys_(0) {
  }

  template<class FACTORGRAPH>
  MetisIndex(const FACTORGRAPH& factorGraph) :
      nKeys_(0) {
    augment(factorGraph);
  }

  ~MetisIndex() {
  }
  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Augment the variable index with new factors.  This can be used when
   * solving problems incrementally.
   */
  template<class FACTORGRAPH>
  void augment(const FACTORGRAPH& factors);

  const std::vector<int32_t>& xadj() const {
    return xadj_;
  }
  const std::vector<int32_t>& adj() const {
    return adj_;
  }
  size_t nValues() const {
    return nKeys_;
  }
  Key intToKey(int32_t value) const {
    assert(value >= 0);
    return intKeyBMap_.right.find(value)->second;
  }

  /// @}
};

} // \ namesace gtsam

#include <gtsam/inference/MetisIndex-inl.h>
