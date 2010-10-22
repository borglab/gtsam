/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SequentialSolver.h
 * @brief   Solves a GaussianFactorGraph (i.e. a sparse linear system) using sequential variable elimination.
 * @author  Richard Roberts
 * @created Oct 19, 2010
 */

#pragma once

#include <gtsam/inference/GenericSequentialSolver.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianConditional.h>

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
 * The EliminationTree recursively produces a BayesNet<GaussianConditional>,
 * typedef'ed in linear/GaussianBayesNet, on which this class calls
 * optimize(...) to perform back-substitution.
 */
class GaussianSequentialSolver : GenericSequentialSolver<GaussianFactor> {

protected:

  typedef GenericSequentialSolver<GaussianFactor> Base;
  typedef boost::shared_ptr<const GaussianSequentialSolver> shared_ptr;

public:

  /**
   * Construct the solver for a factor graph.  This builds the elimination
   * tree, which already does some of the symbolic work of elimination.
   */
  GaussianSequentialSolver(const FactorGraph<GaussianFactor>& factorGraph);

  /**
   * Named constructor to return a shared_ptr.  This builds the elimination
   * tree, which already does some of the symbolic work of elimination.
   */
  static shared_ptr Create(const FactorGraph<GaussianFactor>& factorGraph);

  /**
   * Return a new solver that solves the given factor graph, which must have
   * the *same structure* as the one this solver solves.  For some solvers this
   * is more efficient than constructing the solver from scratch.  This can be
   * used in cases where the numerical values of the linear problem change,
   * e.g. during iterative nonlinear optimization.
   */
  shared_ptr update(const FactorGraph<GaussianFactor>& factorGraph) const;

  /**
   * Eliminate the factor graph sequentially.  Uses a column elimination tree
   * to recursively eliminate.
   */
  GaussianBayesNet::shared_ptr eliminate() const;

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
  std::pair<Vector, Matrix> marginalStandard(Index j) const;

  /**
   * Compute the marginal joint over a set of variables, by integrating out
   * all of the other variables.  This function returns the result as an upper-
   * triangular R factor and right-hand-side, i.e. a GaussianBayesNet with
   * R*x = d.  To get a mean and covariance matrix, use jointStandard(...)
   */
  GaussianFactorGraph::shared_ptr joint(const std::vector<Index>& js) const;

  /**
   * Compute the marginal joint over a set of variables, by integrating out
   * all of the other variables.  This function returns the result as a mean
   * vector and covariance matrix.  The variables will be ordered in the
   * return values as they are ordered in the 'js' argument, not as they are
   * ordered in the original factor graph.  Compared to jointCanonical, which
   * returns a GaussianBayesNet, this function back-substitutes the BayesNet to
   * obtain the mean, then computes \Sigma = (R^T * R)^-1.
   */
//  std::pair<Vector, Matrix> jointStandard(const std::vector<Index>& js) const;
};

}

