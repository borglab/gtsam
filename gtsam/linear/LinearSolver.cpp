/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearOptimizer.cpp
 * @brief   Common Interface for Linear Optimizers
 * @author  Fan Jiang
 */

#include "LinearSolver.h"

namespace gtsam {

  std::unique_ptr<LinearSolver> LinearSolver::fromNonlinearParams(gtsam::NonlinearOptimizerParams nlparams) {
    return std::unique_ptr<LinearSolver>();
  }

  LinearSolver::LinearSolver() {

  }

}