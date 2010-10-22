/**
 * @file    GaussianMultifrontalSolver.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#pragma once

#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <utility>
#include <vector>

namespace gtsam {

/** This solver uses sequential variable elimination to solve a
 * GaussianFactorGraph, i.e. a sparse linear system.  Underlying this is a
 * column elimination tree (inference/EliminationTree), see Gilbert 2001 BIT.
 *
 * The elimination ordering is "baked in" to the variable indices at this
 * stage, i.e. elimination proceeds in order from '0'.  A fill-reducing
 * ordering is computed symbolically from the NonlinearFactorGraph, on the
 * nonlinear side of gtsam.  (To be precise, it is possible to permute an
 * existing GaussianFactorGraph into a COLAMD ordering instead, this is done
 * when computing marginals).
 *
 * This is not the most efficient algorithm we provide, most efficient is the
 * MultifrontalSolver, which performs Multi-frontal QR factorization.  However,
 * sequential variable elimination is easier to understand so this is a good
 * starting point to learn about these algorithms and our implementation.
 * Additionally, the first step of MFQR is symbolic sequential elimination.
 *
 * The EliminationTree recursively produces a BayesNet<GaussianFactor>,
 * typedef'ed in linear/GaussianBayesNet, on which this class calls
 * optimize(...) to perform back-substitution.
 */
class GaussianMultifrontalSolver {

protected:

  GaussianJunctionTree::shared_ptr junctionTree_;

public:

  /**
   * Construct the solver for a factor graph.  This builds the elimination
   * tree, which already does some of the symbolic work of elimination.
   */
  GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>& factorGraph);

  /**
   * Eliminate the factor graph sequentially.  Uses a column elimination tree
   * to recursively eliminate.
   */
  typename BayesTree<GaussianConditional>::sharedClique eliminate() const;

  /**
   * Compute the least-squares solution of the GaussianFactorGraph.  This
   * eliminates to create a BayesNet and then back-substitutes this BayesNet to
   * obtain the solution.
   */
  VectorValues::shared_ptr optimize() const;

  /**
   * Compute the marginal Gaussian density over a variable, by integrating out
   * all of the other variables.  This function returns the result as an upper-
   * triangular R factor and right-hand-side, i.e. a GaussianConditional with
   * R*x = d.  To get a mean and covariance matrix, use marginalStandard(...)
   */
  GaussianFactor::shared_ptr marginal(Index j) const;

  /**
   * Compute the marginal Gaussian density over a variable, by integrating out
   * all of the other variables.  This function returns the result as a mean
   * vector and covariance matrix.  Compared to marginalCanonical, which
   * returns a GaussianConditional, this function back-substitutes the R factor
   * to obtain the mean, then computes \Sigma = (R^T * R)^-1.
   */
//  std::pair<Vector, Matrix> marginalStandard(Index j) const;

};

}


