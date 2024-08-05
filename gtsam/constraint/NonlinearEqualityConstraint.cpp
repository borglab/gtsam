/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    EqualityConstraint.cpp
 * @brief   Equality constraints in constrained optimization.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024 */

#include <gtsam/constraint/NonlinearEqualityConstraint.h>

namespace gtsam {

/* ************************************************************************* */
ZeroCostConstraint::ZeroCostConstraint(const NoiseModelFactor::shared_ptr& factor)
    : Base(constrainedNoise(factor->noiseModel()->sigmas()), factor->keys()), factor_(factor) {}

/* ************************************************************************* */
Vector ZeroCostConstraint::unwhitenedError(const Values& x, OptionalMatrixVecType H) const {
  return factor_->unwhitenedError(x, H);
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr ZeroCostConstraint::penaltyFactor(const double mu) const {
  return factor_->cloneWithNewNoiseModel(penaltyNoise(mu));
}

/* ************************************************************************* */
NonlinearEqualityConstraints NonlinearEqualityConstraints::FromCostGraph(
    const NonlinearFactorGraph& graph) {
  NonlinearEqualityConstraints constraints;
  for (const auto& factor : graph) {
    auto noise_factor = std::dynamic_pointer_cast<NoiseModelFactor>(factor);
    constraints.emplace_shared<ZeroCostConstraint>(noise_factor);
  }
  return constraints;
}

/* ************************************************************************* */
size_t NonlinearEqualityConstraints::dim() const {
  size_t dimension = 0;
  for (const auto& constraint : *this) {
    dimension += constraint->dim();
  }
  return dimension;
}

/* ************************************************************************* */
Vector NonlinearEqualityConstraints::violationVector(const Values& values, bool whiten) const {
  Vector violation(dim());
  size_t start_idx = 0;
  for (const auto& constraint : *this) {
    size_t dim = constraint->dim();
    violation.middleCols(start_idx, dim) =
        whiten ? constraint->whitenedError(values) : constraint->unwhitenedError(values);
    start_idx += dim;
  }
  return violation;
}

/* ************************************************************************* */
double NonlinearEqualityConstraints::violationNorm(const Values& values) const {
  return violationVector(values, true).norm();
}

/* ************************************************************************* */
NonlinearFactorGraph NonlinearEqualityConstraints::penaltyGraph(const double mu) const {
  NonlinearFactorGraph graph;
  for (const auto& constraint : *this) {
    graph.add(constraint->penaltyFactor(mu));
  }
  return graph;
}

}  // namespace gtsam
