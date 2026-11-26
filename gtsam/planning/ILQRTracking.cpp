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
                           const KeyVector& stateKeys,
                           const KeyVector& controlKeys,
                           const ILQRLinearization& lin, double damping) {
  const size_t horizon = controlKeys.size();
  const Eigen::Index h = static_cast<Eigen::Index>(horizon);
  const Eigen::Index m = lin.B.front().cols();
  TrackingSweep sweep{GaussianBayesNet(), Matrix::Zero(m, h),
                      std::vector<Matrix>(horizon)};
  std::vector<GaussianConditional::shared_ptr> conditionals(horizon);

  HessianFactor valueToGo = costs.terminalFactor;

  for (Eigen::Index k = h - 1; k >= 0; --k) {
    const auto idx = static_cast<size_t>(k);
    HessianFactor stageFactor = pullBackDynamics(
        stateKeys[idx], controlKeys[idx], lin.A[idx], lin.B[idx], valueToGo);

    for (const auto& factor : costs.stageFactors[idx])
      factor->updateHessian(stageFactor.keys(), &stageFactor.info());
    if (damping > 0.0)
      stageFactor.info().updateDiagonalBlock(0,
                                             damping * Matrix::Identity(m, m));

    auto [conditional, nextValueToGo] =
        eliminateControlFactor(std::move(stageFactor), controlKeys[idx]);
    if (!conditional) return {};
    extractControlLaw(*conditional, sweep.feedforward.col(k), sweep.gains[idx]);
    conditionals[idx] = std::move(conditional);
    valueToGo = std::move(nextValueToGo);
  }

  sweep.policy.reserve(horizon);
  for (auto& cond : conditionals) sweep.policy.push_back(std::move(cond));
  return sweep;
}

}  // namespace internal
}  // namespace gtsam
