/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CuSparseSolver.h
 *
 * @brief CuSparse based linear solver backend for GTSAM
 *
 * @date Jun 2020
 * @author Fan Jiang
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/LinearSolver.h>
#include <string>

namespace gtsam {

  /**
   * cuSparse based Backend class
   */
  class GTSAM_EXPORT CuSparseSolver : public LinearSolver {
  public:

    typedef enum {
      QR,
      CHOLESKY
    } CuSparseSolverType;


    explicit CuSparseSolver(CuSparseSolver::CuSparseSolverType type, const Ordering &ordering);

    bool isIterative() override;

    bool isSequential() override;

    VectorValues solve(const GaussianFactorGraph &gfg) override;

  protected:

    CuSparseSolverType solverType = QR;

    Ordering ordering;
  };
}  // namespace gtsam
