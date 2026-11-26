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

StageLinearizedCosts linearizeStageCosts(const NonlinearFactorGraph& cost,
                                         const Values& values,
                                         const KeyVector& stateKeys,
                                         const KeyVector& controlKeys,
                                         const StageKeyMap& keyMap,
                                         Eigen::Index stateDim) {
  const size_t h = controlKeys.size();
  StageLinearizedCosts out{
      std::vector<GaussianFactorGraph>(h),
      HessianFactor(stateKeys[h], Matrix::Zero(stateDim, stateDim),
                    Vector::Zero(stateDim), 0.0)};

  const auto linearCost = cost.linearize(values);
  for (const auto& factor : *linearCost) {
    if (!factor || factor->empty()) continue;
    const auto& keys = factor->keys();
    if (keys.size() == 1) {
      const auto it = keyMap.find(keys[0]);
      if (it == keyMap.end())
        throw std::invalid_argument(
            "NonlinearFactorGraph contains a key outside state/control keys.");
      const auto [stage, isControl] = it->second;
      if (!isControl && stage == h)
        factor->updateHessian(out.terminalFactor.keys(),
                              &out.terminalFactor.info());
      else
        out.stageFactors[stage].push_back(factor);
    } else if (keys.size() == 2) {
      const auto it0 = keyMap.find(keys[0]);
      const auto it1 = keyMap.find(keys[1]);
      if (it0 == keyMap.end() || it1 == keyMap.end() ||
          it0->second.first != it1->second.first ||
          it0->second.second == it1->second.second)
        throw std::invalid_argument(
            "Chain trajectory optimizer requires unary stage/terminal factors "
            "or binary (X(k), U(k)) factors.");
      out.stageFactors[it0->second.first].push_back(factor);
    } else {
      throw std::invalid_argument(
          "Chain trajectory optimizer supports unary stage/terminal factors "
          "and binary (X(k), U(k)) factors.");
    }
  }
  return out;
}

HessianFactor pullBackDynamics(Key stateKey, Key controlKey, const Matrix& A,
                               const Matrix& B,
                               const HessianFactor& nextValueToGo) {
  const Matrix Vxx = nextValueToGo.information();
  const Vector vx = nextValueToGo.linearTerm();
  const Matrix VxxA = Vxx * A;
  const Matrix VxxB = Vxx * B;
  return HessianFactor(controlKey, stateKey, B.transpose() * VxxB,
                       B.transpose() * VxxA, B.transpose() * vx,
                       A.transpose() * VxxA, A.transpose() * vx,
                       nextValueToGo.constantTerm());
}

std::pair<GaussianConditional::shared_ptr, HessianFactor>
eliminateControlFactor(HessianFactor stageFactor, Key controlKey) {
  try {
    auto conditional = stageFactor.eliminateCholesky(Ordering{controlKey});
    return {std::move(conditional), std::move(stageFactor)};
  } catch (const IndeterminateSystemException&) {
    return {nullptr, HessianFactor()};
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
