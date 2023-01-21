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
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/LinearSolverParams.h>
#include <gtsam/linear/VectorValues.h>

#include <string>

namespace gtsam {

/**
 * Variable elimination based linear solver backend wrapper for GTSAM.
 * This class is a wrapper for factor graph elimination methods to follow the
 * "LinearSolver" unified API.
 */
class GTSAM_EXPORT EliminationSolver : public LinearSolver {
 protected:
  LinearSolverParams params_;

 public:
  explicit EliminationSolver(const LinearSolverParams &params)
      : params_(params) {};

  /**
   * Solve the Gaussian factor graph using variable elimination.
   * @param gfg the factor graph to solve
   * @returns the solution
   */
  VectorValues solve(const GaussianFactorGraph &gfg) const override {
    switch (params_.linearSolverType) {
      case LinearSolverParams::MULTIFRONTAL_QR:
        return params_.ordering ? gfg.optimize(*params_.ordering, EliminateQR)
                                : gfg.optimize(EliminateQR);
      case LinearSolverParams::MULTIFRONTAL_CHOLESKY:
        return params_.ordering
                   ? gfg.optimize(*params_.ordering, EliminatePreferCholesky)
                   : gfg.optimize(EliminatePreferCholesky);
      case LinearSolverParams::SEQUENTIAL_QR:
        return params_.ordering
                   ? gfg.eliminateSequential(*params_.ordering, EliminateQR,
                                             boost::none, params_.orderingType)
                         ->optimize()
                   : gfg.eliminateSequential(EliminateQR, boost::none,
                                             params_.orderingType)
                         ->optimize();
      case LinearSolverParams::SEQUENTIAL_CHOLESKY:
        return params_.ordering
                   ? gfg.eliminateSequential(*params_.ordering,
                                             EliminatePreferCholesky,
                                             boost::none, params_.orderingType)
                         ->optimize()
                   : gfg.eliminateSequential(EliminatePreferCholesky,
                                             boost::none, params_.orderingType)
                         ->optimize();
      default:
        throw std::runtime_error(
            "EliminationSolver::solve: Solver type is invalid for "
            "EliminationSolver");
    }
  };
};

}  // namespace gtsam
