/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearOptimizerState.h
 * @brief Private class for NonlinearOptimizer state
 * @author Richard Roberts
 * @author Frank Dellaert
 * @date Sep 7, 2009
 */

#pragma once

#include <gtsam/nonlinear/Values.h>

namespace gtsam {
namespace internal {

/**
 * Base class for a nonlinear optimization state, including the current estimate
 * of the variable values, error, and number of iterations.  Optimizers derived
 * from NonlinearOptimizer usually also define a derived state class containing
 * additional state specific to the algorithm (for example, Dogleg state
 * contains the current trust region radius).
 */
struct NonlinearOptimizerState {
 public:
  /** The current estimate of the variable values. */
  const Values values;

  /** The factor graph error on the current values. */
  const double error;

  /** The number of optimization iterations performed. */
  const size_t iterations;

  virtual ~NonlinearOptimizerState() {}

  NonlinearOptimizerState(const Values& values, double error, size_t iterations = 0)
      : values(values), error(error), iterations(iterations) {}

  // Constructor version that takes ownership of values
  NonlinearOptimizerState(Values&& values, double error, size_t iterations = 0)
      : values(std::move(values)), error(error), iterations(iterations) {}
};

}  // namespace internal
}  // namespace gtsam
