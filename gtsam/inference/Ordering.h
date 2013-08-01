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

    /// Compute a fill-reducing ordering using COLAMD from a factor graph (see details for note on
    /// performance). This internally builds a VariableIndex so if you already have a VariableIndex,
    /// it is faster to use COLAMD(const VariableIndex&)
    template<class FACTOR>
    static Ordering COLAMD(const FactorGraph<FACTOR>& graph) {
      return COLAMD(VariableIndex(graph)); }

    /// Compute a fill-reducing ordering using COLAMD from a VariableIndex.
    static GTSAM_EXPORT Ordering COLAMD(const VariableIndex& variableIndex);

    /// Compute a fill-reducing ordering using constrained COLAMD from a factor graph (see details
    /// for note on performance).  This internally builds a VariableIndex so if you already have a
    /// VariableIndex, it is faster to use COLAMD(const VariableIndex&).  This function constrains
    /// the variables in \c constrainLast to the end of the ordering, and orders all other variables
    /// before in a fill-reducing ordering.  If \c forceOrder is true, the variables in \c
    /// constrainLast will be ordered in the same order specified in the vector<Key> \c
    /// constrainLast.   If \c constrainLast is false, the variables in \c constrainLast will be
    /// ordered after all the others, but will be rearranged by CCOLAMD to reduce fill-in as well.
    template<class FACTOR>
    static Ordering COLAMDConstrainedLast(const FactorGraph<FACTOR>& graph,
      const std::vector<Key>& constrainLast, bool forceOrder = false) {
        return COLAMDConstrainedLast(VariableIndex(graph), constrainLast, forceOrder); }

    /// Compute a fill-reducing ordering using constrained COLAMD from a VariableIndex.  This
    /// function constrains the variables in \c constrainLast to the end of the ordering, and orders
    /// all other variables before in a fill-reducing ordering.  If \c forceOrder is true, the
    /// variables in \c constrainLast will be ordered in the same order specified in the vector<Key>
    /// \c constrainLast.   If \c constrainLast is false, the variables in \c constrainLast will be
    /// ordered after all the others, but will be rearranged by CCOLAMD to reduce fill-in as well.
    static GTSAM_EXPORT Ordering COLAMDConstrainedLast(const VariableIndex& variableIndex,
      const std::vector<Key>& constrainLast, bool forceOrder = false);

    /// Compute a fill-reducing ordering using constrained COLAMD from a factor graph (see details
    /// for note on performance).  This internally builds a VariableIndex so if you already have a
    /// VariableIndex, it is faster to use COLAMD(const VariableIndex&).  In this function, a group
    /// for each variable should be specified in \c groups, and each group of variables will appear
    /// in the ordering in group index order.  \c groups should be a map from Key to group index.
    /// The group indices used should be consecutive starting at 0, but may appear in \c groups in
    /// arbitrary order.  Any variables not present in \c groups will be assigned to group 0.  This
    /// function simply fills the \c cmember argument to CCOLAMD with the supplied indices, see the
    /// CCOLAMD documentation for more information.
    template<class FACTOR>
    static Ordering COLAMDConstrained(const FactorGraph<FACTOR>& graph,
      const FastMap<Key, int>& groups) {
        return COLAMDConstrained(VariableIndex(graph), groups); }

    /// Compute a fill-reducing ordering using constrained COLAMD from a VariableIndex.  In this
    /// function, a group for each variable should be specified in \c groups, and each group of
    /// variables will appear in the ordering in group index order.  \c groups should be a map from
    /// Key to group index. The group indices used should be consecutive starting at 0, but may
    /// appear in \c groups in arbitrary order.  Any variables not present in \c groups will be
    /// assigned to group 0.  This function simply fills the \c cmember argument to CCOLAMD with the
    /// supplied indices, see the CCOLAMD documentation for more information.
    static GTSAM_EXPORT Ordering COLAMDConstrained(const VariableIndex& variableIndex,
      const FastMap<Key, int>& groups);

  private:
    static GTSAM_EXPORT Ordering COLAMDConstrained(
      const VariableIndex& variableIndex, std::vector<int>& cmember);
  };
}

