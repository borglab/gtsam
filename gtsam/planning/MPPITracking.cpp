// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file MPPITracking.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#include <gtsam/planning/MPPITracking.h>

namespace gtsam {

Matrix smoothControlTrajectory(const Matrix& controls, const Matrix& history) {
  if (!controls.rows() || !controls.cols() || !controls.allFinite() ||
      history.rows() != controls.rows() || history.cols() != 2 ||
      !history.allFinite())
    throw std::invalid_argument(
        "Expected a trajectory and its two preceding controls.");

  // Pad left with the two preceding controls and right by repeating the end.
  Matrix extended(controls.rows(), controls.cols() + 4);
  extended << history, controls, controls.rightCols(1), controls.rightCols(1);

  // Convolve each step with the 5-point quadratic Savitzky-Golay kernel.
  const Vector5 coefficients = Vector5{-3., 12., 17., 12., -3.} / 35.;
  Matrix smoothed(controls.rows(), controls.cols());
  for (Eigen::Index k = 0; k < controls.cols(); ++k)
    smoothed.col(k) = extended.middleCols(k, 5) * coefficients;
  return smoothed;
}

TsallisMPPI::TsallisMPPI(double cutoff, double shape, double inverseTemperature)
    : cutoff_(cutoff), shape_(shape), beta_(inverseTemperature) {
  if (!(cutoff > 0) || !(shape > 1) || !std::isfinite(cutoff) ||
      !std::isfinite(shape) || !(beta_ > 0) || !std::isfinite(beta_))
    throw std::invalid_argument(
        "Tsallis MPPI requires a positive cutoff and r > 1.");
}

Vector TsallisMPPI::weights(const Vector& costs) const {
  if (!costs.size() || costs.hasNaN() || !std::isfinite(costs.minCoeff()))
    throw std::domain_error("Expected a finite minimum trajectory cost.");

  // Truncate costs beyond min + cutoff to zero before normalizing.
  Vector weights = (1.0 - (costs.array() - costs.minCoeff()) / cutoff_)
                       .max(0.0)
                       .pow(1.0 / (shape_ - 1.0));
  return weights / weights.sum();
}

}  // namespace gtsam
