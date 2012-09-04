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
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

/* ************************************************************************* */
VectorValues optimize(const GaussianBayesTree& bayesTree) {
  VectorValues result = *allocateVectorValues(bayesTree);
  optimizeInPlace(bayesTree, result);
  return result;
}

/* ************************************************************************* */
void optimizeInPlace(const GaussianBayesTree& bayesTree, VectorValues& result) {
  internal::optimizeInPlace<GaussianBayesTree>(bayesTree.root(), result);
}

/* ************************************************************************* */
VectorValues optimizeGradientSearch(const GaussianBayesTree& bayesTree) {
  tic(0, "Allocate VectorValues");
  VectorValues grad = *allocateVectorValues(bayesTree);
  toc(0, "Allocate VectorValues");

  optimizeGradientSearchInPlace(bayesTree, grad);

  return grad;
}

/* ************************************************************************* */
void optimizeGradientSearchInPlace(const GaussianBayesTree& bayesTree, VectorValues& grad) {
  tic(1, "Compute Gradient");
  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  gradientAtZero(bayesTree, grad);
  double gradientSqNorm = grad.dot(grad);
  toc(1, "Compute Gradient");

  tic(2, "Compute R*g");
  // Compute R * g
  FactorGraph<JacobianFactor> Rd_jfg(bayesTree);
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
VectorValues gradient(const GaussianBayesTree& bayesTree, const VectorValues& x0) {
  return gradient(FactorGraph<JacobianFactor>(bayesTree), x0);
}

/* ************************************************************************* */
void gradientAtZero(const GaussianBayesTree& bayesTree, VectorValues& g) {
  gradientAtZero(FactorGraph<JacobianFactor>(bayesTree), g);
}

}




