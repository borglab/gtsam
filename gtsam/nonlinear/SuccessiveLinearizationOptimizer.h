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
  /** See SuccessiveLinearizationParams::elimination */
  enum Elimination {
    MULTIFRONTAL,
    SEQUENTIAL
  };

  /** See SuccessiveLinearizationParams::factorization */
  enum Factorization {
    CHOLESKY,
    QR,
  };

  Elimination elimination; ///< The elimination algorithm to use (default: MULTIFRONTAL)
  Factorization factorization; ///< The numerical factorization (default: Cholesky)
  boost::optional<Ordering> ordering; ///< The variable elimination ordering, or empty to use COLAMD (default: empty)

  SuccessiveLinearizationParams() :
    elimination(MULTIFRONTAL), factorization(CHOLESKY) {}

  virtual ~SuccessiveLinearizationParams() {}

  virtual void print(const std::string& str = "") const {
    NonlinearOptimizerParams::print(str);
    if(elimination == MULTIFRONTAL)
      std::cout << "         elimination method: MULTIFRONTAL\n";
    else if(elimination == SEQUENTIAL)
      std::cout << "         elimination method: SEQUENTIAL\n";
    else
      std::cout << "         elimination method: (invalid)\n";

    if(factorization == CHOLESKY)
      std::cout << "       factorization method: CHOLESKY\n";
    else if(factorization == QR)
      std::cout << "       factorization method: QR\n";
    else
      std::cout << "       factorization method: (invalid)\n";

    if(ordering)
      std::cout << "                   ordering: custom\n";
    else
      std::cout << "                   ordering: COLAMD\n";

    std::cout.flush();
  }

  GaussianFactorGraph::Eliminate getEliminationFunction() const {
    if(factorization == SuccessiveLinearizationParams::CHOLESKY)
      return EliminatePreferCholesky;
    else if(factorization == SuccessiveLinearizationParams::QR)
      return EliminateQR;
    else
      throw runtime_error("Nonlinear optimization parameter \"factorization\" is invalid");
  }
};

} /* namespace gtsam */
