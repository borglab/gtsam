/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VariableIndex.h
 * @author  Richard Roberts
 * @date    March 26, 2013
 */

#pragma once

#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/dllexport.h>

#include <boost/optional/optional.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <cassert>
#include <stdexcept>
#include <optional>

namespace gtsam {

/**
 * The VariableIndex class computes and stores the block column structure of a
 * factor graph.  The factor graph stores a collection of factors, each of
 * which involves a set of variables.  In contrast, the VariableIndex is built
 * from a factor graph prior to elimination, and stores the list of factors
 * that involve each variable.  This information is stored as a deque of
 * lists of factor indices.
 * \nosubgrouping
 */
class GTSAM_EXPORT VariableIndex {
 public:
  typedef boost::shared_ptr<VariableIndex> shared_ptr;
  typedef FactorIndices::iterator Factor_iterator;
  typedef FactorIndices::const_iterator Factor_const_iterator;

 protected:
  typedef FastMap<Key, FactorIndices> KeyMap;
  KeyMap index_;
  size_t nFactors_;  // Number of factors in the original factor graph.
  size_t nEntries_;  // Sum of involved variable counts of each factor.

 public:
  typedef KeyMap::const_iterator const_iterator;
  typedef KeyMap::const_iterator iterator;
  typedef KeyMap::value_type value_type;

  /// @name Standard Constructors
  /// @{

  /// Default constructor, creates an empty VariableIndex
  VariableIndex() : nFactors_(0), nEntries_(0) {}

  /**
   * Create a VariableIndex that computes and stores the block column structure
   * of a factor graph.
   */
  template <class FG>
  explicit VariableIndex(const FG& factorGraph) : nFactors_(0), nEntries_(0) {
    augment(factorGraph);
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// The number of variable entries.  This is equal to the number of unique variable Keys.
  size_t size() const { return index_.size(); }

  /// The number of factors in the original factor graph
  size_t nFactors() const { return nFactors_; }

  /// The number of nonzero blocks, i.e. the number of variable-factor entries
  size_t nEntries() const { return nEntries_; }

  /// Access a list of factors by variable
  const FactorIndices& operator[](Key variable) const {
    KeyMap::const_iterator item = index_.find(variable);
    if(item == index_.end())
      throw std::invalid_argument("Requested non-existent variable from VariableIndex");
    else
    return item->second;
  }

  /// Return true if no factors associated with a variable
  bool empty(Key variable) const {
    return (*this)[variable].empty();
  }

  /// @}
  /// @name Testable
  /// @{

  /// Test for equality (for unit tests and debug assertions).
  bool equals(const VariableIndex& other, double tol=0.0) const;

  /// Print the variable index (for unit tests and debugging).
  void print(const std::string& str = "VariableIndex: ",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /**
   * Output dual hypergraph to Metis file format for use with hmetis
   * In the dual graph, variables are hyperedges, factors are nodes.
   */
  void outputMetisFormat(std::ostream& os) const;


  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Augment the variable index with new factors.  This can be used when
   * solving problems incrementally.
   */
  template<class FG>
  void augment(const FG& factors, const FactorIndices* newFactorIndices = nullptr);

  /**
   * An overload of augment() that takes a single factor. and l-value
   * reference to FactorIndeces.
   */
  template<class FG>
  void augment(const FG& factor, const FactorIndices& newFactorIndices) {
    augment(factor, &newFactorIndices);
  }

  /**
   * Augment the variable index after an existing factor now affects to more
   * variable Keys. This can be used when solving problems incrementally, with
   * smart factors or in general with factors with a dynamic number of Keys.
   */
  void augmentExistingFactor(const FactorIndex factorIndex, const KeySet & newKeys);

  /**
   * Remove entries corresponding to the specified factors. NOTE: We intentionally do not decrement
   * nFactors_ because the factor indices need to remain consistent.  Removing factors from a factor
   * graph does not shift the indices of other factors.  Also, we keep nFactors_ one greater than
   * the highest-numbered factor referenced in a VariableIndex.
   *
   * @param indices The indices of the factors to remove, which must match \c factors
   * @param factors The factors being removed, which must symbolically correspond exactly to the
   *        factors with the specified \c indices that were added.
   */
  template<typename ITERATOR, class FG>
  void remove(ITERATOR firstFactor, ITERATOR lastFactor, const FG& factors);

  /// Remove unused empty variables (in debug mode verifies they are empty).
  template<typename ITERATOR>
  void removeUnusedVariables(ITERATOR firstKey, ITERATOR lastKey);

  /// Iterator to the first variable entry
  const_iterator begin() const { return index_.begin(); }

  /// Iterator to the first variable entry
  const_iterator end() const { return index_.end(); }

  /// Find the iterator for the requested variable entry
  const_iterator find(Key key) const { return index_.find(key); }

protected:
  Factor_iterator factorsBegin(Key variable) { return internalAt(variable).begin(); }
  Factor_iterator factorsEnd(Key variable) { return internalAt(variable).end(); }

  Factor_const_iterator factorsBegin(Key variable) const { return internalAt(variable).begin(); }
  Factor_const_iterator factorsEnd(Key variable) const { return internalAt(variable).end(); }

  /// Internal version of 'at' that asserts existence
  const FactorIndices& internalAt(Key variable) const {
    const KeyMap::const_iterator item = index_.find(variable);
    assert(item != index_.end());
    return item->second; 
  }

  /// Internal version of 'at' that asserts existence
  FactorIndices& internalAt(Key variable) {
    const KeyMap::iterator item = index_.find(variable);
    assert(item != index_.end());
    return item->second; 
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(index_);
    ar & BOOST_SERIALIZATION_NVP(nFactors_);
    ar & BOOST_SERIALIZATION_NVP(nEntries_);
  }

  /// @}
};

/// traits
template<>
struct traits<VariableIndex> : public Testable<VariableIndex> {
};

} //\ namespace gtsam

#include <gtsam/inference/VariableIndex-inl.h>
