/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesTree.cpp
 * @brief   Gaussian Bayes Tree, the result of eliminating a GaussianJunctionTree
 * @brief   GaussianBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/inference/BayesTreeUnordered-inst.h>
#include <gtsam/inference/BayesTreeCliqueBaseUnordered-inst.h>
#include <gtsam/linear/GaussianBayesTreeUnordered.h>
#include <gtsam/linear/GaussianFactorGraphUnordered.h>
#include <gtsam/linear/GaussianBayesNetUnordered.h>
#include <gtsam/linear/VectorValuesUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  namespace internal
  {
    /* ************************************************************************* */
    /** Pre-order visitor for back-substitution in a Bayes tree.  The visitor function operator()()
     *  optimizes the clique given the solution for the parents, and returns the solution for the
     *  clique's frontal variables.  In addition, it adds the solution to a global collected
     *  solution that will finally be returned to the user.  The reason we pass the individual
     *  clique solutions between nodes is to avoid log(n) lookups over all variables, they instead
     *  then are only over a node's parent variables. */
    struct OptimizeClique
    {
      VectorValuesUnordered collectedResult;

      VectorValuesUnordered operator()(
        const GaussianBayesTreeCliqueUnordered::shared_ptr& clique,
        const VectorValuesUnordered& parentSolution)
      {
        // parents are assumed to already be solved and available in result
        VectorValuesUnordered cliqueSolution = clique->conditional()->solve(parentSolution);
        collectedResult.insert(cliqueSolution);
        return cliqueSolution;
      }
    };

    /* ************************************************************************* */
    double logDeterminant(const GaussianBayesTreeCliqueUnordered::shared_ptr& clique, double& parentSum)
    {
      parentSum += clique->conditional()->get_R().diagonal().unaryExpr(std::ptr_fun<double,double>(log)).sum();
    }
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianBayesTreeUnordered::optimize() const {
    internal::OptimizeClique visitor;
    VectorValuesUnordered empty;
    treeTraversal::DepthFirstForest(*this, empty, visitor);
    return visitor.collectedResult;
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianBayesTreeUnordered::optimizeGradientSearch() const
  {
    gttic(Compute_Gradient);
    // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
    VectorValuesUnordered grad;
    bayesTree.gradientAtZeroInPlace(grad);
    double gradientSqNorm = grad.dot(grad);
    gttoc(Compute_Gradient);

    gttic(Compute_Rg);
    // Compute R * g
    Errors Rg = GaussianFactorGraphUnordered(*this) * grad;
    gttoc(Compute_Rg);

    gttic(Compute_minimizing_step_size);
    // Compute minimizing step size
    double step = -gradientSqNorm / dot(Rg, Rg);
    gttoc(Compute_minimizing_step_size);

    gttic(Compute_point);
    // Compute steepest descent point
    scal(step, grad);
    gttoc(Compute_point);

    return grad;
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianBayesTreeUnordered::gradient(const VectorValuesUnordered& x0) const {
    return gtsam::gradient(GaussianFactorGraphUnordered(*this), x0);
  }

  /* ************************************************************************* */
  void GaussianBayesTreeUnordered::gradientAtZeroInPlace(VectorValuesUnordered& g) const {
    gradientAtZero(GaussianFactorGraphUnordered(*this), g);
  }

  /* ************************************************************************* */
  double GaussianBayesTreeUnordered::logDeterminant() const
  {
    if(this->roots_.empty()) {
      return 0.0;
    } else {
      double sum = 0.0;
      treeTraversal::DepthFirstForest(*this, sum, internal::logDeterminant);
      return sum;
    }
  }

  /* ************************************************************************* */
  double GaussianBayesTreeUnordered::determinant() const
  {
    return exp(logDeterminant());
  }

} // \namespace gtsam




