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

namespace gtsam {

class SuccessiveLinearizationParams : public NonlinearOptimizerParams {
public:
  /** See SuccessiveLinearizationParams::linearSolverType */
  enum LinearSolverType {
    MULTIFRONTAL_CHOLESKY,
    MULTIFRONTAL_QR,
    SEQUENTIAL_CHOLESKY,
    SEQUENTIAL_QR,
    SPCG
  };

	LinearSolverType linearSolverType; ///< The type of linear solver to use in the nonlinear optimizer
  boost::optional<Ordering> ordering; ///< The variable elimination ordering, or empty to use COLAMD (default: empty)

  SuccessiveLinearizationParams() : linearSolverType(MULTIFRONTAL_CHOLESKY) {}

  virtual ~SuccessiveLinearizationParams() {}

  virtual void print(const std::string& str = "") const {
    NonlinearOptimizerParams::print(str);
    switch ( linearSolverType ) {
    case MULTIFRONTAL_CHOLESKY:
      std::cout << "         linear solver type: MULTIFRONTAL CHOLESKY\n";
      break;
    case MULTIFRONTAL_QR:
      std::cout << "         linear solver type: MULTIFRONTAL QR\n";
      break;
    case SEQUENTIAL_CHOLESKY:
      std::cout << "         linear solver type: SEQUENTIAL CHOLESKY\n";
      break;
    case SEQUENTIAL_QR:
      std::cout << "         linear solver type: SEQUENTIAL QR\n";
      break;
    case SPCG:
      std::cout << "         linear solver type: SPCG\n";
      break;
    default:
      std::cout << "         linear solver type: (invalid)\n";
      break;
    }

    if(ordering)
      std::cout << "                   ordering: custom\n";
    else
      std::cout << "                   ordering: COLAMD\n";

    std::cout.flush();
  }

  inline bool isMultifrontal() const {
    return (linearSolverType == MULTIFRONTAL_CHOLESKY) || (linearSolverType == MULTIFRONTAL_QR);
  }

  inline bool isSequential() const {
    return (linearSolverType == SEQUENTIAL_CHOLESKY) || (linearSolverType == SEQUENTIAL_QR);
  }

  inline bool isSPCG() const {
    return (linearSolverType == SPCG);
  }

  GaussianFactorGraph::Eliminate getEliminationFunction() {
    switch (linearSolverType) {
    case MULTIFRONTAL_CHOLESKY:
    case MULTIFRONTAL_QR:
      return EliminatePreferCholesky;

    case SEQUENTIAL_CHOLESKY:
    case SEQUENTIAL_QR:
      return EliminateQR;

    default:
      throw runtime_error("Nonlinear optimization parameter \"factorization\" is invalid");
      break;
    }
  }
};

} /* namespace gtsam */
