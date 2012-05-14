/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Marginals.h
 * @brief A class for computing marginals in a NonlinearFactorGraph
 * @author Richard Roberts
 * @date May 14, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

/**
 * A class for computing Gaussian marginals of variables in a NonlinearFactorGraph
 */
class Marginals {

public:

  /** The linear factorization mode - either CHOLESKY (faster and suitable for most problems) or QR (slower but more numerically stable for poorly-conditioned problems). */
  enum Factorization {
    CHOLESKY,
    QR
  };

  /** Construct a marginals class.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The linearization point about which to compute Gaussian marginals (usually the MLE as obtained from a NonlinearOptimizer).
   * @param factorization The linear decomposition mode - either Marginals::CHOLESKY (faster and suitable for most problems) or Marginals::QR (slower but more numerically stable for poorly-conditioned problems).
   */
  Marginals(const NonlinearFactorGraph& graph, const Values& solution, Factorization factorization = CHOLESKY);

  /** Compute the marginal covariance of a single variable */
  Matrix marginalCovariance(Key variable) const;

  /** Compute the marginal information matrix of a single variable.  You can
   * use LLt(const Matrix&) or RtR(const Matrix&) to obtain the square-root information
   * matrix. */
  Matrix marginalInformation(Key variable) const;

protected:

  GaussianFactorGraph graph_;
  Ordering ordering_;
  Factorization factorization_;
  GaussianBayesTree bayesTree_;

};

} /* namespace gtsam */
