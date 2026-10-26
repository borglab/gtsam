/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file GaussianElimination.h
 * @brief Preserve an input Hessian diagonal across one elimination operation.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam::internal {

/** Numerical state for one Gaussian elimination operation. Copies share the
 * immutable input diagonal. Returned factors carry no reference metadata. */
class GTSAM_EXPORT GaussianElimination {
 public:
  using Eliminate = GaussianFactorGraph::Eliminate;

  /// Start an operation, or share the operation already carried by function.
  GaussianElimination(const GaussianFactorGraph& graph, const Eliminate& function);

  /// Execute one frontal elimination with this operation's numerical policy.
  GaussianFactorGraph::EliminationResult operator()(
      const GaussianFactorGraph& factors, const Ordering& keys) const;

  /// Adapt the operation to a generic traversal's ordinary callback interface.
  Eliminate function() const { return *this; }

 private:
  Eliminate function_;
  std::shared_ptr<const VectorValues> reference_;
  bool preferCholesky_ = false;
};

}  // namespace gtsam::internal
