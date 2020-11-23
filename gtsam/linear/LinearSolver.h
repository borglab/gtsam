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
 * @author  Fan Jiang, Gerry Chen, Mandy Xie, and Frank Dellaert
 */

#pragma once

#include <gtsam/linear/LinearSolverParams.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

/** Common Interface Class for all linear solvers */
class GTSAM_EXPORT LinearSolver {
 protected:
  // default constructor needed in order to delete copy constructor
  LinearSolver() = default;
 public:
  LinearSolver(LinearSolver &) = delete;

  /**
   * Solve a Gaussian Factor Graph with the solver
   * @param gfg the GFG to be optimized
   * @return the optimization result in VectorValues
   */
  virtual VectorValues solve(const GaussianFactorGraph &gfg) const = 0;

  /**
   * Alias for `solve`
   * @param gfg the GFG to be optimized
   * @return the optimization result in VectorValues
   */
  VectorValues operator()(const GaussianFactorGraph &gfg) const {
    return solve(gfg);
  }

  /**
   * Factory method for generating a derived class of LinearSolver from
   * LinearSolverParams
   * @param params LinearSolverParams linear optimizer parameters
   * @return pointer to a LinearSolver-derived object
   */
  static boost::shared_ptr<LinearSolver> CreateFromParameters(
      const LinearSolverParams &params);
};

}  // namespace gtsam
