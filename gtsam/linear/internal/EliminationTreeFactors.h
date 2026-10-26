/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file EliminationTreeFactors.h
 * @brief Collect tree input factors to establish one elimination reference.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/FastVector.h>

namespace gtsam::internal {

/// Collect the numerical input of a concrete Gaussian or hybrid tree.
template <class TREE>
typename TREE::FactorGraphType eliminationTreeFactors(const TREE& tree) {
  typename TREE::FactorGraphType graph;
  graph.push_back(tree.remainingFactors());
  FastVector<typename TREE::sharedNode> pending(tree.roots());
  while (!pending.empty()) {
    const auto node = pending.back();
    pending.pop_back();
    graph.push_back(node->factors);
    pending.insert(pending.end(), node->children.begin(), node->children.end());
  }
  return graph;
}

}  // namespace gtsam::internal
