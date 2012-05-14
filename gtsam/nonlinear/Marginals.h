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
 * A class for computing marginals in a NonlinearFactorGraph
 */
class Marginals {

public:

  enum Factorization {
    CHOLESKY,
    QR
  };

  Marginals(const NonlinearFactorGraph& graph, const Values& solution, Factorization factorization = CHOLESKY);

  Matrix marginalCovariance(Key variable) const;

protected:

  GaussianFactorGraph graph_;
  Ordering ordering_;
  Factorization factorization_;
  GaussianBayesTree bayesTree_;

};

} /* namespace gtsam */
