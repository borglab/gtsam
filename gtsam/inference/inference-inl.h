/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   inference-inl.h
 * @brief  inference template definitions
 * @author Frank Dellaert, Richard Roberts
 */

#pragma once

#include <boost/foreach.hpp>

#include <gtsam/inference/inference.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
template<typename CONSTRAINED>
Permutation::shared_ptr Inference::PermutationCOLAMD(const VariableIndex& variableIndex, const CONSTRAINED& constrainLast) {

  vector<int> cmember(variableIndex.size(), 0);

  // If at least some variables are not constrained to be last, constrain the
  // ones that should be constrained.
  if(constrainLast.size() < variableIndex.size()) {
    BOOST_FOREACH(Index var, constrainLast) {
      assert(var < variableIndex.size());
      cmember[var] = 1;
    }
  }

  return PermutationCOLAMD_(variableIndex, cmember);
}

/* ************************************************************************* */
inline Permutation::shared_ptr Inference::PermutationCOLAMD(const VariableIndex& variableIndex) {
  vector<int> cmember(variableIndex.size(), 0);
  return PermutationCOLAMD_(variableIndex, cmember);
}

} // namespace gtsam
