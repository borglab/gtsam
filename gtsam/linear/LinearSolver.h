/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearSolver.h
 * @brief   Common Interface for Linear Solvers
 * @author  Fan Jiang
 */

#pragma once

#include <gtsam/linear/LinearSolverParams.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

namespace gtsam {
  class LinearSolver {
  public:
    LinearSolver(LinearSolver &) = delete;

    gtsam::LinearSolverType linearSolverType = MULTIFRONTAL_CHOLESKY; ///< The type of linear solver to use in the nonlinear optimizer

    virtual bool isIterative() = 0;

    virtual bool isSequential() = 0;

    static std::unique_ptr<LinearSolver> fromNonlinearParams(gtsam::NonlinearOptimizerParams nlparams);

    virtual VectorValues solve(const GaussianFactorGraph &gfg) = 0;

  protected:
    LinearSolver();
  };

}