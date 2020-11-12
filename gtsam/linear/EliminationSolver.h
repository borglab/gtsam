/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EliminationSolver.h
 *
 * @brief Variable elimination based linear solver backend wrapper for GTSAM.
 * This class is just a wrapper for factor graph elimination methods to follow
 * the "LinearSolver" unified API.
 *
 * @date Nov 2020
 * @author Gerry Chen
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/LinearSolverParams.h>
#include <gtsam/linear/VectorValues.h>

#include <string>

namespace gtsam {

/**
 * variable elimination based linear solver wrapper class
 */
class GTSAM_EXPORT EliminationSolver : public LinearSolver {
 public:
  explicit EliminationSolver(const LinearSolverParams &params)
      : params_(params){};

  bool isIterative() override { return false; };

  bool isSequential() override {
    return params_.linearSolverType == LinearSolverParams::SEQUENTIAL_QR ||
           params_.linearSolverType == LinearSolverParams::SEQUENTIAL_CHOLESKY;
  };

  VectorValues solve(const GaussianFactorGraph &gfg) override;

 protected:
  LinearSolverParams params_;
};

}  // namespace gtsam
