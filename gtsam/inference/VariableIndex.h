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

#include <gtsam/inference/Key.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/dllexport.h>

#include <boost/optional/optional.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <cassert>
#include <stdexcept>

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
  typedef FastVector<size_t> Factors;
  typedef Factors::iterator Factor_iterator;
  typedef Factors::const_iterator Factor_const_iterator;

protected:
  typedef FastMap<Key,Factors> KeyMap;
  KeyMap index_;
  size_t nFactors_; // Number of factors in the original factor graph.
  size_t nEntries_; // Sum of involved variable counts of each factor.

public:
  typedef KeyMap::const_iterator const_iterator;
  typedef KeyMap::const_iterator iterator;
  typedef KeyMap::value_type value_type;

public:

  /// @name Standard Constructors
  /// @{

  /** Default constructor, creates an empty VariableIndex */
  VariableIndex() : nFactors_(0), nEntries_(0) {}

  /**
   * Create a VariableIndex that computes and stores the block column structure
   * of a factor graph.
   */
  template<class FG>
  VariableIndex(const FG& factorGraph) : nFactors_(0), nEntries_(0) { augment(factorGraph); }

  /// @}
  /// @name Standard Interface
  /// @{

  /**
   * The number of variable entries.  This is one greater than the variable
   * with the highest index.
   */
  size_t size() const { return index_.size(); }

  /** The number of factors in the original factor graph */
  size_t nFactors() const { return nFactors_; }

  /** The number of nonzero blocks, i.e. the number of variable-factor entries */
  size_t nEntries() const { return nEntries_; }

  /** Access a list of factors by variable */
  const Factors& operator[](Key variable) const {
    KeyMap::const_iterator item = index_.find(variable);
    if(item == index_.end())
      throw std::invalid_argument("Requested non-existent variable from VariableIndex");
    else
    return item->second;
  }

  /// @}
  /// @name Testable
  /// @{

  /** Test for equality (for unit tests and debug assertions). */
  bool equals(const VariableIndex& other, double tol=0.0) const;

  /** Print the variable index (for unit tests and debugging). */
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
  void augment(const FG& factors, boost::optional<const FastVector<size_t>&> newFactorIndices = boost::none);

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

  /** Remove unused empty variables (in debug mode verifies they are empty). */
  template<typename ITERATOR>
  void removeUnusedVariables(ITERATOR firstKey, ITERATOR lastKey);

  /** Iterator to the first variable entry */
  const_iterator begin() const { return index_.begin(); }

  /** Iterator to the first variable entry */
  const_iterator end() const { return index_.end(); }

  /** Find the iterator for the requested variable entry */
  const_iterator find(Key key) const { return index_.find(key); }

protected:
  Factor_iterator factorsBegin(Key variable) { return internalAt(variable).begin(); }
  Factor_iterator factorsEnd(Key variable) { return internalAt(variable).end(); }

  Factor_const_iterator factorsBegin(Key variable) const { return internalAt(variable).begin(); }
  Factor_const_iterator factorsEnd(Key variable) const { return internalAt(variable).end(); }

  /// Internal version of 'at' that asserts existence
  const Factors& internalAt(Key variable) const {
    const KeyMap::const_iterator item = index_.find(variable);
    assert(item != index_.end());
    return item->second; }

  /// Internal version of 'at' that asserts existence
  Factors& internalAt(Key variable) {
    const KeyMap::iterator item = index_.find(variable);
    assert(item != index_.end());
    return item->second; }

  /// @}
};

/// traits
template<>
struct traits<VariableIndex> : public Testable<VariableIndex> {
};

} //\ namespace gtsam

#include <gtsam/inference/VariableIndex-inl.h>
