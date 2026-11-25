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

}  // namespace

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
  std::partial_sort(order.begin(), order.begin() + elites_, order.end(),
                    [&](auto a, auto b) {
                      return costs[a] == costs[b] ? a < b : costs[a] < costs[b];
                    });
  Vector weights = Vector::Zero(costs.size());
  for (size_t i = 0; i < elites_ && std::isfinite(costs[order[i]]); ++i)
    weights[order[i]] = 1;
  return weights / weights.sum();
}

}  // namespace gtsam
