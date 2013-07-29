/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.h
 * @author  Richard Roberts
 * @date    Sep 2, 2010
 */

#pragma once

#include <vector>

#include <gtsam/inference/Key.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/FactorGraph.h>

namespace gtsam {
  class Ordering : public std::vector<Key> {
  protected:
    typedef std::vector<Key> Base;

  public:
    /// Create an empty ordering
    GTSAM_EXPORT Ordering() {}

    /// Create from a container
    template<typename KEYS>
    explicit Ordering(const KEYS& keys) : Base(keys.begin(), keys.end()) {}

    /// Create an ordering using iterators over keys
    template<typename ITERATOR>
    Ordering(ITERATOR firstKey, ITERATOR lastKey) : Base(firstKey, lastKey) {}

    /// Invert (not reverse) the ordering - returns a map from key to order position
    FastMap<Key, size_t> invert() const;

    /// Compute an ordering using COLAMD directly from a factor graph - this internally builds a
    /// VariableIndex so if you already have a VariableIndex, it is faster to use COLAMD(const
    /// VariableIndex&)
    template<class FACTOR>
    static Ordering COLAMD(const FactorGraph<FACTOR>& graph) {
      return COLAMD(VariableIndex(graph));
    }

    static GTSAM_EXPORT Ordering COLAMD(const VariableIndex& variableIndex);

    static GTSAM_EXPORT Ordering COLAMDConstrainedLast(
      const VariableIndex& variableIndex, const std::vector<Key>& constrainLast, bool forceOrder = false);

  private:
    static GTSAM_EXPORT Ordering COLAMDConstrained(
      const VariableIndex& variableIndex,
      std::vector<int>& cmember);
  };
}

