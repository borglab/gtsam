// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file    SamplingAlgorithms.cpp
 * @author  Frank Dellaert
 * @author  Akash Ratheesh
 */

#include <gtsam/planning/SamplingAlgorithms.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam {
namespace {

void checkCosts(const Vector& costs) {
  if (!costs.size())
    throw std::invalid_argument("Expected at least one candidate.");
  if (costs.hasNaN() || !std::isfinite(costs.minCoeff()))
    throw std::domain_error(
        "Expected finite costs or +infinity, with at least one finite cost.");
}

double checkedTemperature(double beta) {
  if (!std::isfinite(beta) || beta < 0)
    throw std::invalid_argument(
        "Expected a finite nonnegative inverse temperature.");
  return beta;
}

Vector exponentialWeights(const Vector& costs, double beta,
                          const Vector& offsets) {
  checkCosts(costs);

  // Shift by minimum cost for numerical stability and add optional log offsets.
  Vector logs = Vector::Zero(costs.size());
  if (beta != 0) logs = -beta * (costs.array() - costs.minCoeff());
  if (offsets.size()) {
    if (offsets.size() != costs.size() || offsets.hasNaN() ||
        offsets.maxCoeff() == std::numeric_limits<double>::infinity())
      throw std::invalid_argument(
          "Expected one finite or negative infinite log offset per candidate.");
    logs += offsets;
  }

  // Force infeasible candidates to zero weight even when beta == 0.
  for (Eigen::Index i = 0; i < costs.size(); ++i)
    if (!std::isfinite(costs[i]))
      logs[i] = -std::numeric_limits<double>::infinity();

  const double maxLog = logs.maxCoeff();
  if (!std::isfinite(maxLog))
    throw std::domain_error("All candidates have zero weight.");

  // Exponentiate, normalize, and flush subnormal weights to zero.
  Vector weights = (logs.array() - maxLog).exp();
  weights /= weights.sum();
  return (weights.array() < std::numeric_limits<double>::min())
      .select(0.0, weights);
}

double empiricalRisk(Vector costs, double alpha, bool averageTail) {
  if (!costs.size() || !costs.allFinite() || !(alpha >= 0. && alpha <= 1.))
    throw std::invalid_argument(
        "Expected finite scenario costs and a quantile in [0, 1].");

  // Linearly interpolate the empirical alpha-quantile on sorted costs.
  std::sort(costs.data(), costs.data() + costs.size());
  const double index = alpha * static_cast<double>(costs.size() - 1);
  const auto lower = static_cast<Eigen::Index>(std::floor(index));
  const auto upper = static_cast<Eigen::Index>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  const double quantile =
      (1. - fraction) * costs[lower] + fraction * costs[upper];
  if (!averageTail) return quantile;

  // Average the interpolated quantile with all observations at/above ceil.
  const double weight = 1. / static_cast<double>(1 + costs.size() - upper);
  return weight * quantile + (weight * costs.tail(costs.size() - upper)).sum();
}

}  // namespace

double valueAtRisk(const Vector& costs, double alpha) {
  return empiricalRisk(costs, alpha, false);
}

double conditionalValueAtRisk(const Vector& costs, double alpha) {
  return empiricalRisk(costs, alpha, true);
}

MPPI::MPPI(double beta) : inverseTemperature_(checkedTemperature(beta)) {}
Vector MPPI::weights(const Vector& costs, const Vector& offsets) const {
  return exponentialWeights(costs, inverseTemperature_, offsets);
}

Vector PredictiveSampling::weights(const Vector& costs) const {
  checkCosts(costs);
  Eigen::Index best;
  costs.minCoeff(&best);
  return Vector::Unit(costs.size(), best);
}

CEM::CEM(size_t elites, double floor) : elites_(elites), minStddev_(floor) {
  if (!elites || !(floor > 0) || !std::isfinite(floor))
    throw std::invalid_argument(
        "Expected a positive elite count and spread floor.");
}

Vector CEM::weights(const Vector& costs) const {
  checkCosts(costs);
  if (static_cast<size_t>(costs.size()) < elites_)
    throw std::invalid_argument(
        "CEM needs at least as many candidates as elites.");
  std::vector<Eigen::Index> order(costs.size());
  std::iota(order.begin(), order.end(), 0);
  std::partial_sort(order.begin(),
                    order.begin() + static_cast<std::ptrdiff_t>(elites_),
                    order.end(), [&](auto a, auto b) {
                      return costs[a] == costs[b] ? a < b : costs[a] < costs[b];
                    });
  Vector weights = Vector::Zero(costs.size());
  for (size_t i = 0; i < elites_ && std::isfinite(costs[order[i]]); ++i)
    weights[order[i]] = 1;
  return weights / weights.sum();
}

}  // namespace gtsam
