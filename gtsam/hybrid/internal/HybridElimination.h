/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtsam/hybrid/HybridGaussianFactorGraph.h>

namespace gtsam::internal {

/** Numerical state for one hybrid operation, with hypothesis-specific scales. */
class GTSAM_EXPORT HybridElimination {
 public:
  using Eliminate = HybridGaussianFactorGraph::Eliminate;

  /** Start an operation, or share the operation already carried by function. */
  HybridElimination(const HybridGaussianFactorGraph& graph,
                    const Eliminate& function);

  /** Execute one frontal elimination under this operation's hypothesis policy. */
  HybridGaussianFactorGraph::EliminationResult operator()(
      const HybridGaussianFactorGraph& factors, const Ordering& keys) const {
    return function_(factors, keys);
  }

  /** Adapt the operation to a generic traversal's ordinary callback interface. */
  Eliminate function() const { return *this; }

 private:
  Eliminate function_;
};

}  // namespace gtsam::internal
