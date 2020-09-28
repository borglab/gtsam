/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GlobalConstraintSequentialSolver.h
 *
 * @brief Sequential elimination-based solver for GTSAM that can efficiently
 *        deal with global constraints
 *
 * @date Sept 2020
 * @author Gerry Chen
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/LinearSolver.h>

namespace gtsam {

/**
 * Global constraints backend class, using sequential elimination
 */
class GTSAM_EXPORT GlobalConstraintSequentialSolver : public LinearSolver {
 protected:
  LinearSolverParams params_;

 public:
  explicit GlobalConstraintSequentialSolver(const LinearSolverParams &params)
      : params_(params) {}

  bool isIterative() override { return false; };

  bool isSequential() override { return true; };

  VectorValues solve(const GaussianFactorGraph &gfg) override;
};
}  // namespace gtsam
