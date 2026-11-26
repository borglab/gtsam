// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file ILQRTracking.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <gtsam/planning/ILQRTracking.h>

#include <utility>

namespace gtsam {
namespace internal {

TrackingSweep backwardILQR(const StageLinearizedCosts& costs,
                           const ILQRLinearization& lin, double damping) {
  const size_t horizon = costs.stageFactors.size();
  const Eigen::Index h = static_cast<Eigen::Index>(horizon);
  const Eigen::Index m = lin.B.front().cols();
  TrackingSweep sweep{GaussianBayesNet(), Matrix::Zero(m, h),
                      std::vector<Matrix>(horizon)};
  sweep.policy.resize(horizon);

  HessianFactor valueToGo = costs.terminalFactor;

  for (Eigen::Index k = h - 1; k >= 0; --k) {
    const auto idx = static_cast<size_t>(k);
    auto conditional = eliminateControlStage(
        lin.A[idx], lin.B[idx], costs.stageFactors[idx], damping, valueToGo);
    if (!conditional) return {};
    extractControlLaw(*conditional, sweep.feedforward.col(k), sweep.gains[idx]);
    sweep.policy[idx] = std::move(conditional);
  }

  return sweep;
}

}  // namespace internal
}  // namespace gtsam
