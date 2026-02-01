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

#include <vector>
#include <map>
#include <unordered_map>
#include <iostream>

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
  std::vector<int32_t> adj_;  // Stores adjacency lists of all nodes, appended into a single vector
  BiMap intKeyBMap_; // Stores Key <-> integer value relationship
  size_t nKeys_;

public:
  /// @name Constructors
  /// @{

  /** Default constructor, creates empty MetisIndex */
  MetisIndex() : nKeys_(0) {}

  template<class FactorGraphType>
  MetisIndex(const FactorGraphType& factorGraph) :
    nKeys_(0) {
    augment(factorGraph);
  }

  ~MetisIndex() {}

  /// @}
  /// @name Standard API
  /// @{
    
  /**
   * Augment the variable index with new factors.  This can be used when
   * solving problems incrementally.
   */
  template<class FactorGraphType>
  void augment(const FactorGraphType& factors);
  
  const std::vector<int32_t>& xadj() const { return xadj_; }
  const std::vector<int32_t>& adj() const { return adj_; }
  size_t nValues() const { return nKeys_; }
  
  Key intToKey(int32_t value) const { return intKeyBMap_.right.find(value)->second; }
  
  /// @}
  /// @name Testable
  /// @{
    
  /// print to std::cout
  void print(const std::string& str = "MetisIndex:") const {
    std::cout << str << "\nxadj_: ";
    for (const auto& x : xadj_) std::cout << x << " ";
    std::cout << "\nadj_: ";
    for (const auto& x : adj_) std::cout << x << " ";
    std::cout << "\nKey <-> Index: ";
    for (const auto& [i, k] : intKeyBMap_.left) std::cout << i << " <-> " << k << ", ";
    std::cout << std::endl;
  }

  /// @}
};

} // \ namespace gtsam

#include <gtsam/inference/MetisIndex-inl.h>
