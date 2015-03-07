/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianMultifrontalSolver.h
 * @author  Richard Roberts
 * @date    Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/GenericMultifrontalSolver.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/VectorValues.h>

#include <utility>

namespace gtsam {

/**
 * This solver uses multifrontal elimination to solve a GaussianFactorGraph,
 * i.e. a sparse linear system.  Underlying this is a junction tree, which is
 * eliminated into a Bayes tree.
 *
 * The elimination ordering is "baked in" to the variable indices at this
 * stage, i.e. elimination proceeds in order from '0'.  A fill-reducing
 * ordering is computed symbolically from the NonlinearFactorGraph, on the
 * nonlinear side of gtsam.  (It is actually also possible to permute an
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

  /** flag to determine whether to use Cholesky or QR */
  bool useQR_;

public:

  /**
   * Construct the solver for a factor graph.  This builds the junction
   * tree, which already does some of the work of elimination.
   */
  GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>& factorGraph, bool useQR = false);

  /**
   * Construct the solver with a shared pointer to a factor graph and to a
   * VariableIndex.  The solver will store these pointers, so this constructor
   * is the fastest.
   */
  GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph,
  		const VariableIndex::shared_ptr& variableIndex, bool useQR = false);

  /**
   * Named constructor to return a shared_ptr.  This builds the elimination
   * tree, which already does some of the symbolic work of elimination.
   */
  static shared_ptr Create(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph,
  		const VariableIndex::shared_ptr& variableIndex, bool useQR = false);

  /**
   * Return a new solver that solves the given factor graph, which must have
   * the *same structure* as the one this solver solves.  For some solvers this
   * is more efficient than constructing the solver from scratch.  This can be
   * used in cases where the numerical values of the linear problem change,
   * e.g. during iterative nonlinear optimization.
   */
  void replaceFactors(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph);

  /**
   * Eliminate the factor graph sequentially.  Uses a column elimination tree
   * to recursively eliminate.
   */
  GaussianBayesTree::shared_ptr eliminate() const;

  /**
   * Compute the least-squares solution of the GaussianFactorGraph.  This
   * eliminates to create a BayesNet and then back-substitutes this BayesNet to
   * obtain the solution.
   */
  VectorValues::shared_ptr optimize() const;

  /**
   * Compute the marginal joint over a set of variables, by integrating out
   * all of the other variables.  This function returns the result as a factor
   * graph.
   *
   * NOTE: This function is limited to computing a joint on 2 variables
   */
  GaussianFactorGraph::shared_ptr jointFactorGraph(const std::vector<Index>& js) const;

  /**
   * Compute the marginal Gaussian density over a variable, by integrating out
   * all of the other variables.  This function returns the result as an upper-
   * triangular R factor and right-hand-side, i.e. a GaussianConditional with
   * R*x = d.  To get a mean and covariance matrix, use marginalStandard(...)
   */
  GaussianFactor::shared_ptr marginalFactor(Index j) const;

  /**
   * Compute the marginal Gaussian density over a variable, by integrating out
   * all of the other variables.  This function returns the result as a mean
   * vector and covariance matrix.  Compared to marginalCanonical, which
   * returns a GaussianConditional, this function back-substitutes the R factor
   * to obtain the mean, then computes \f$ \Sigma = (R^T * R)^{-1} \f$.
   */
   Matrix marginalCovariance(Index j) const;

   /** @return true if using QR */
   inline bool usingQR() const { return useQR_; }

};

}


