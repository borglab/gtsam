#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

#include <algorithm>
#include <cmath>

namespace gtsam::internal {

/** Increase LM damping after a rejected linear solve or trial step. */
inline void increaseLevenbergMarquardtLambda(
    const LevenbergMarquardtParams& params, double* lambda,
    double* currentFactor) {
  *lambda *= *currentFactor;
  if (!params.useFixedLambdaFactor) {
    *currentFactor *= 2.0;
  }
}

/** Decrease LM damping after accepting a step with the given fidelity. */
inline void decreaseLevenbergMarquardtLambda(
    const LevenbergMarquardtParams& params, double modelFidelity,
    double* lambda, double* currentFactor) {
  if (params.useFixedLambdaFactor) {
    *lambda /= *currentFactor;
  } else {
    *lambda *= std::max(
        1.0 / 3.0,
        1.0 - std::pow(2.0 * modelFidelity - 1.0, 3));
    *currentFactor *= 2.0;
  }
  *lambda = std::max(params.lambdaLowerBound, *lambda);
}

}  // namespace gtsam::internal
