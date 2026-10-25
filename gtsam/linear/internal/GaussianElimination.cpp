/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <gtsam/linear/internal/GaussianElimination.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

namespace gtsam::internal {

GaussianElimination::GaussianElimination(const GaussianFactorGraph& graph,
                                         const Eliminate& function)
    : function_(function) {
  if (const auto* operation = function.target<GaussianElimination>()) {
    *this = *operation;
    return;
  }
  if (!usesCholesky(function) || hasConstraints(graph)) return;
  preferCholesky_ = !function.target<decltype(&EliminateCholesky)>();
  reference_ = std::make_shared<const VectorValues>(graph.hessianDiagonal());
}

GaussianFactorGraph::EliminationResult GaussianElimination::operator()(
    const GaussianFactorGraph& factors, const Ordering& keys) const {
  if (!reference_) return function_(factors, keys);
  if (preferCholesky_ && hasConstraints(factors)) return EliminateQR(factors, keys);
  return eliminateCholeskyWithReference(factors, keys, reference_->vector(keys));
}

}  // namespace gtsam::internal
