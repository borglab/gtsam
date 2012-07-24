/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SuccessiveLinearizationOptimizer.h
 * @brief 
 * @author Richard Roberts
 * @date Apr 1, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/linear/IterativeSolver.h>

namespace gtsam {

class SuccessiveLinearizationParams : public NonlinearOptimizerParams {
public:
  /** See SuccessiveLinearizationParams::linearSolverType */
  enum LinearSolverType {
    MULTIFRONTAL_CHOLESKY,
    MULTIFRONTAL_QR,
    SEQUENTIAL_CHOLESKY,
    SEQUENTIAL_QR,
    CG,         /* Experimental Flag */
    CHOLMOD,    /* Experimental Flag */
  };

	LinearSolverType linearSolverType; ///< The type of linear solver to use in the nonlinear optimizer
  boost::optional<Ordering> ordering; ///< The variable elimination ordering, or empty to use COLAMD (default: empty)
  IterativeOptimizationParameters::shared_ptr iterativeParams; ///< The container for iterativeOptimization parameters. used in CG Solvers.

  SuccessiveLinearizationParams() : linearSolverType(MULTIFRONTAL_CHOLESKY) {}
  virtual ~SuccessiveLinearizationParams() {}

  inline bool isMultifrontal() const {
    return (linearSolverType == MULTIFRONTAL_CHOLESKY) || (linearSolverType == MULTIFRONTAL_QR);
  }

  inline bool isSequential() const {
    return (linearSolverType == SEQUENTIAL_CHOLESKY) || (linearSolverType == SEQUENTIAL_QR);
  }

  inline bool isCholmod() const {
    return (linearSolverType == CHOLMOD);
  }

  inline bool isCG() const {
    return (linearSolverType == CG);
  }

  virtual void print(const std::string& str) const;

  GaussianFactorGraph::Eliminate getEliminationFunction() const {
    switch (linearSolverType) {
    case MULTIFRONTAL_CHOLESKY:
    case SEQUENTIAL_CHOLESKY:
      return EliminatePreferCholesky;

    case MULTIFRONTAL_QR:
    case SEQUENTIAL_QR:
      return EliminateQR;

    default:
      throw std::runtime_error("Nonlinear optimization parameter \"factorization\" is invalid");
      return EliminateQR;
      break;
    }
  }
};

/* a wrapper for solving a GaussianFactorGraph according to the parameters */
VectorValues solveGaussianFactorGraph(const GaussianFactorGraph &gfg, const SuccessiveLinearizationParams &params) ;

} /* namespace gtsam */
