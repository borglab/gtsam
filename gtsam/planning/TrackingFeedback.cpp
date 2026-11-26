// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file TrackingFeedback.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <gtsam/planning/TrackingFeedback.h>

#include <Eigen/Cholesky>
#include <cmath>
#include <limits>
#include <stdexcept>

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
