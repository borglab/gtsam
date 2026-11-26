// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file TrackingFeedback.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/planning/TrackingFeedback.h>

#include <Eigen/Cholesky>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace gtsam {
namespace internal {

void checkTrackingPrecision(const Matrix& precision, bool definite) {
  if (!precision.rows() || precision.rows() != precision.cols() ||
      !precision.allFinite() || !precision.isApprox(precision.transpose()))
    throw std::invalid_argument(
        "Expected a finite symmetric precision matrix.");
  Eigen::LDLT<Matrix> ldlt(precision);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive() ||
      (definite && (ldlt.vectorD().array() <= 0).any()))
    throw std::invalid_argument("Invalid tracking precision.");
}

void validateControlLimits(Eigen::Index controlDim, const Vector& lower,
                           const Vector& upper) {
  if ((lower.size() &&
       (lower.size() != controlDim || lower.hasNaN() ||
        (lower.array() == std::numeric_limits<double>::infinity()).any())) ||
      (upper.size() &&
       (upper.size() != controlDim || upper.hasNaN() ||
        (upper.array() == -std::numeric_limits<double>::infinity()).any())) ||
      (lower.size() && upper.size() && (lower.array() > upper.array()).any()))
    throw std::invalid_argument("Invalid tracking control limits.");
}

StageKeyMap buildStageKeyMap(const KeyVector& stateKeys,
                             const KeyVector& controlKeys) {
  const size_t h = controlKeys.size();
  if (!h || stateKeys.size() != h + 1)
    throw std::invalid_argument(
        "Expected H control keys and H + 1 state keys.");
  StageKeyMap keyMap;
  keyMap.reserve(2 * h + 1);
  for (size_t k = 0; k <= h; ++k) {
    if (!keyMap.emplace(stateKeys[k], std::pair{k, false}).second)
      throw std::invalid_argument("Duplicate key in stateKeys.");
  }
  for (size_t k = 0; k < h; ++k) {
    if (!keyMap.emplace(controlKeys[k], std::pair{k, true}).second)
      throw std::invalid_argument("Duplicate key in controlKeys.");
  }
  return keyMap;
}

GaussianConditional::shared_ptr eliminateControlStage(
    const Matrix& A, const Matrix& B, const HessianFactor& stageCost,
    double damping, HessianFactor& valueToGo,
    const HessianFactor* dynamicsCurvature) {
  // Substitute the linearized transition into the separator factor on X(k+1).
  HessianFactor joint(stageCost);  // keys [U(k), X(k)]
  Matrix F(A.rows(), B.cols() + A.cols());
  F << B, A;
  const Matrix projected = valueToGo.informationView() * F;
  joint.info().updateDiagonalBlockAt(0, (F.transpose() * projected).eval());
  joint.linearTerm().noalias() += F.transpose() * valueToGo.linearTerm();
  joint.constantTerm() += valueToGo.constantTerm();
  if (dynamicsCurvature) dynamicsCurvature->updateHessian(&joint);
  if (damping > 0.0)
    joint.info().addToDiagonalBlock(0, Vector::Constant(B.cols(), damping));

  try {
    auto conditional = joint.eliminateCholesky(Ordering{joint.front()});
    valueToGo = std::move(joint);
    return conditional;
  } catch (const IndeterminateSystemException&) {
    return nullptr;
  }
}

void extractControlLaw(const GaussianConditional& conditional,
                       Eigen::Ref<Vector> feedforward, Matrix& gain) {
  const auto R = conditional.R().triangularView<Eigen::Upper>();
  feedforward = R.solve(conditional.d());
  gain = -R.solve(conditional.S());
}

}  // namespace internal

Vector contractionFeedback(const Vector& error, const Matrix& metric,
                           const Matrix& inputJacobian,
                           const Vector& driftDifference, double rate) {
  internal::checkTrackingPrecision(metric, true);
  if (!(rate > 0) || !std::isfinite(rate) || error.size() != metric.rows() ||
      driftDifference.size() != error.size() ||
      inputJacobian.rows() != error.size() || !inputJacobian.cols() ||
      !error.allFinite() || !inputJacobian.allFinite() ||
      !driftDifference.allFinite())
    throw std::invalid_argument(
        "Invalid contraction tracking dimensions or rate.");
  const Vector dual = metric * error;
  const Vector lhs = 2 * inputJacobian.transpose() * dual;
  const double rhs = -2 * dual.dot(rate * error + driftDifference);
  if (rhs >= 0 || lhs.squaredNorm() == 0)
    return Vector::Zero(inputJacobian.cols());
  return (rhs / lhs.squaredNorm()) * lhs;
}

}  // namespace gtsam
