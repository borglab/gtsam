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

namespace gtsam {

  // TODO: Remove this enum
  /** See NonlinearOptimizerParams::linearSolverType */
  typedef enum LinearSolverType {
    MULTIFRONTAL_CHOLESKY,
    MULTIFRONTAL_QR,
    SEQUENTIAL_CHOLESKY,
    SEQUENTIAL_QR,
    Iterative, /* Experimental Flag */
    CHOLMOD, /* Experimental Flag */
    EIGEN_QR,
    EIGEN_CHOLESKY,
  } LinearSolverType;
}


class LinearSolverParams {

};