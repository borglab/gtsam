/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearSolverParams.h
 * @brief   Parameters base class forLinear Solvers
 * @author  Fan Jiang
 */

#pragma once

#include <gtsam/inference/Ordering.h>

#include <boost/optional.hpp>

namespace gtsam {

// Type of solver
typedef enum LinearSolverType {
  MULTIFRONTAL_CHOLESKY,
  MULTIFRONTAL_QR,
  SEQUENTIAL_CHOLESKY,
  SEQUENTIAL_QR,
  Iterative, /* Experimental Flag */
  CHOLMOD,   /* Experimental Flag */
  EIGEN_QR,
  EIGEN_CHOLESKY,
} LinearSolverType;

struct LinearSolverParams {
  LinearSolverType solverType = MULTIFRONTAL_CHOLESKY;
  boost::optional<Ordering> ordering;
};

}  // namespace gtsam