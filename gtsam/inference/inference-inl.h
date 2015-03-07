/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file inference-inl.h
 * @brief 
 * @author Richard Roberts
 * @date Mar 3, 2012
 */

#pragma once

#include <algorithm>

// Only for Eclipse parser, inference-inl.h (this file) is included at the bottom of inference.h
#include <gtsam/inference/inference.h>

#include <gtsam/base/FastSet.h>

namespace gtsam {

namespace inference {

/* ************************************************************************* */
template<typename CONSTRAINED>
Permutation::shared_ptr PermutationCOLAMD(
    const VariableIndex& variableIndex, const CONSTRAINED& constrainLast, bool forceOrder) {
  gttic(PermutationCOLAMD_constrained);

  size_t n = variableIndex.size();
  std::vector<int> cmember(n, 0);

  // If at least some variables are not constrained to be last, constrain the
  // ones that should be constrained.
  if(constrainLast.size() < n) {
    BOOST_FOREACH(Index var, constrainLast) {
      assert(var < n);
      cmember[var] = 1;
    }
  }

  Permutation::shared_ptr permutation = PermutationCOLAMD_(variableIndex, cmember);
  if (forceOrder) {
    Index j=n;
    BOOST_REVERSE_FOREACH(Index c, constrainLast)
      permutation->operator[](--j) = c;
  }
  return permutation;
}

/* ************************************************************************* */
template<typename CONSTRAINED_MAP>
Permutation::shared_ptr PermutationCOLAMDGrouped(
    const VariableIndex& variableIndex, const CONSTRAINED_MAP& constraints) {
  gttic(PermutationCOLAMD_grouped);
  size_t n = variableIndex.size();
  std::vector<int> cmember(n, 0);

  typedef typename CONSTRAINED_MAP::value_type constraint_pair;
  BOOST_FOREACH(const constraint_pair& p, constraints) {
    assert(p.first < n);
    // FIXME: check that no groups are skipped
    cmember[p.first] = p.second;
  }

  return PermutationCOLAMD_(variableIndex, cmember);
}

/* ************************************************************************* */
inline Permutation::shared_ptr PermutationCOLAMD(const VariableIndex& variableIndex) {
  gttic(PermutationCOLAMD_unconstrained);
  size_t n = variableIndex.size();
  std::vector<int> cmember(n, 0);
  return PermutationCOLAMD_(variableIndex, cmember);
}

} // namespace inference
} // namespace gtsam


