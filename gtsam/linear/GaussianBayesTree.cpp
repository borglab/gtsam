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

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/* ************************************************************************* */
namespace internal {
void optimizeInPlace(const boost::shared_ptr<BayesTreeClique<GaussianConditional> >& clique, VectorValues& result) {
  // parents are assumed to already be solved and available in result
  clique->conditional()->solveInPlace(result);

  // starting from the root, call optimize on each conditional
  BOOST_FOREACH(const boost::shared_ptr<BayesTreeClique<GaussianConditional> >& child, clique->children_)
    optimizeInPlace(child, result);
}
}

/* ************************************************************************* */
VectorValues optimize(const GaussianBayesTree& bayesTree) {
  VectorValues result = *allocateVectorValues(bayesTree);
  internal::optimizeInPlace(bayesTree.root(), result);
  return result;
}

/* ************************************************************************* */
VectorValues optimizeGradientSearch(const GaussianBayesTree& Rd) {
  tic(0, "Allocate VectorValues");
  VectorValues grad = *allocateVectorValues(Rd);
  toc(0, "Allocate VectorValues");

  optimizeGradientSearchInPlace(Rd, grad);

  return grad;
}

/* ************************************************************************* */
void optimizeGradientSearchInPlace(const GaussianBayesTree& Rd, VectorValues& grad) {
  tic(1, "Compute Gradient");
  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  gradientAtZero(Rd, grad);
  double gradientSqNorm = grad.dot(grad);
  toc(1, "Compute Gradient");

  tic(2, "Compute R*g");
  // Compute R * g
  FactorGraph<JacobianFactor> Rd_jfg(Rd);
  Errors Rg = Rd_jfg * grad;
  toc(2, "Compute R*g");

  tic(3, "Compute minimizing step size");
  // Compute minimizing step size
  double step = -gradientSqNorm / dot(Rg, Rg);
  toc(3, "Compute minimizing step size");

  tic(4, "Compute point");
  // Compute steepest descent point
  scal(step, grad);
  toc(4, "Compute point");
}

/* ************************************************************************* */
void optimizeInPlace(const GaussianBayesTree& bayesTree, VectorValues& result) {
  internal::optimizeInPlace(bayesTree.root(), result);
}

/* ************************************************************************* */
VectorValues gradient(const GaussianBayesTree& bayesTree, const VectorValues& x0) {
  return gradient(FactorGraph<JacobianFactor>(bayesTree), x0);
}

/* ************************************************************************* */
void gradientAtZero(const GaussianBayesTree& bayesTree, VectorValues& g) {
  gradientAtZero(FactorGraph<JacobianFactor>(bayesTree), g);
}

}




