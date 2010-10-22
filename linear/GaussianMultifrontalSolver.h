/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianMultifrontalSolver.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/GenericMultifrontalSolver.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <utility>
#include <vector>

namespace gtsam {

/** This solver uses multifrontal elimination to solve a GaussianFactorGraph,
 * i.e. a sparse linear system.  Underlying this is a junction tree, which is
 * eliminated into a Bayes tree.
 *
 * The elimination ordering is "baked in" to the variable indices at this
 * stage, i.e. elimination proceeds in order from '0'.  A fill-reducing
 * ordering is computed symbolically from the NonlinearFactorGraph, on the
 * nonlinear side of gtsam.  (To be precise, it is possible to permute an
 * existing GaussianFactorGraph into a COLAMD ordering instead, this is done
 * when computing marginals).
 *
 * The JunctionTree recursively produces a BayesTree<GaussianConditional>,
 * on which this class calls optimize(...) to perform back-substitution.
 */
class GaussianMultifrontalSolver : GenericMultifrontalSolver<GaussianFactor, GaussianJunctionTree> {

protected:

  typedef GenericMultifrontalSolver<GaussianFactor, GaussianJunctionTree> Base;
  typedef boost::shared_ptr<const GaussianMultifrontalSolver> shared_ptr;

public:

  /**
   * Construct the solver for a factor graph.  This builds the elimination
   * tree, which already does some of the symbolic work of elimination.
   */
  GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>& factorGraph);

  /**
   * Named constructor that returns a shared_ptr.  This builds the junction
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
  BayesTree<GaussianConditional>::shared_ptr eliminate() const;

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

};

}


