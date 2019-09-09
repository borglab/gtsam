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
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

  /** Common Interface Class for all linear solvers */
  class LinearSolver {
  public:
    LinearSolver(LinearSolver &) = delete;

    // TODO: Remove this and use trait functions instead?
    gtsam::LinearSolverType linearSolverType = MULTIFRONTAL_CHOLESKY; ///< The type of this instance

    virtual bool isIterative() {
      return false;
    };

    virtual bool isSequential() {
      return false;
    };

    ///
    /**
     * Factor method for generating a LinearSolver from legacy NonlinearOptimizerParams
     * @param nonlinear optimizer parameters
     * @return pointer to a LinearSolver object
     */
    static std::shared_ptr<LinearSolver> fromNonlinearParams(const gtsam::NonlinearOptimizerParams &nlparams);

    virtual VectorValues solve(const GaussianFactorGraph &gfg, const Ordering &ordering) {
      throw std::runtime_error(
          "BUG_CHECK: Calling solve of the base class!");
    };

  protected:
    LinearSolver();

    virtual ~LinearSolver() = default;
  };

}