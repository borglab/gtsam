/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteLookupDAG.cpp
 *  @date Aug, 2022
 *  @author Shangjie Xue
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteLookupDAG.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridLookupDAG.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/linear/VectorValues.h>

#include <string>
#include <utility>

using std::pair;
using std::vector;

namespace gtsam {

/* ************************************************************************** */
void HybridLookupTable::argmaxInPlace(HybridValues* values) const {
  // For discrete conditional, uses argmaxInPlace() method in
  // DiscreteLookupTable.
  if (isDiscrete()) {
    boost::static_pointer_cast<DiscreteLookupTable>(inner_)->argmaxInPlace(
        &(values->discrete));
  } else if (isContinuous()) {
    // For Gaussian conditional, uses solve() method in GaussianConditional.
    values->continuous.insert(
        boost::static_pointer_cast<GaussianConditional>(inner_)->solve(
            values->continuous));
  } else if (isHybrid()) {
    // For hybrid conditional, since children should not contain discrete
    // variable, we can condition on the discrete variable in the parents and
    // solve the resulting GaussianConditional.
    auto conditional =
        boost::static_pointer_cast<GaussianMixture>(inner_)->conditionals()(
            values->discrete);
    values->continuous.insert(conditional->solve(values->continuous));
  }
}

/* ************************************************************************** */
HybridLookupDAG HybridLookupDAG::FromBayesNet(const HybridBayesNet& bayesNet) {
  HybridLookupDAG dag;
  for (auto&& conditional : bayesNet) {
    HybridLookupTable hlt(*conditional);
    dag.push_back(hlt);
  }
  return dag;
}

/* ************************************************************************** */
HybridValues HybridLookupDAG::argmax(HybridValues result) const {
  // Argmax each node in turn in topological sort order (parents first).
  for (auto lookupTable : boost::adaptors::reverse(*this))
    lookupTable->argmaxInPlace(&result);
  return result;
}

}  // namespace gtsam
