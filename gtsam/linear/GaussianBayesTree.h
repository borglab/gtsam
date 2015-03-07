/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesTree.h
 * @brief   Gaussian Bayes Tree, the result of eliminating a GaussianJunctionTree
 * @brief   GaussianBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/inference/BayesTree.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>

namespace gtsam {

/// A Bayes Tree representing a Gaussian density
typedef BayesTree<GaussianConditional> GaussianBayesTree;

/// optimize the BayesTree, starting from the root
VectorValues optimize(const GaussianBayesTree& bayesTree);

/// recursively optimize this conditional and all subtrees
void optimizeInPlace(const GaussianBayesTree& bayesTree, VectorValues& result);

namespace internal {
template<class BAYESTREE>
void optimizeInPlace(const typename BAYESTREE::sharedClique& clique, VectorValues& result);
}

/**
 * Optimize along the gradient direction, with a closed-form computation to
 * perform the line search.  The gradient is computed about \f$ \delta x=0 \f$.
 *
 * This function returns \f$ \delta x \f$ that minimizes a reparametrized
 * problem.  The error function of a GaussianBayesNet is
 *
 * \f[ f(\delta x) = \frac{1}{2} |R \delta x - d|^2 = \frac{1}{2}d^T d - d^T R \delta x + \frac{1}{2} \delta x^T R^T R \delta x \f]
 *
 * with gradient and Hessian
 *
 * \f[ g(\delta x) = R^T(R\delta x - d), \qquad G(\delta x) = R^T R. \f]
 *
 * This function performs the line search in the direction of the
 * gradient evaluated at \f$ g = g(\delta x = 0) \f$ with step size
 * \f$ \alpha \f$ that minimizes \f$ f(\delta x = \alpha g) \f$:
 *
 * \f[ f(\alpha) = \frac{1}{2} d^T d + g^T \delta x + \frac{1}{2} \alpha^2 g^T G g \f]
 *
 * Optimizing by setting the derivative to zero yields
 * \f$ \hat \alpha = (-g^T g) / (g^T G g) \f$.  For efficiency, this function
 * evaluates the denominator without computing the Hessian \f$ G \f$, returning
 *
 * \f[ \delta x = \hat\alpha g = \frac{-g^T g}{(R g)^T(R g)} \f]
 */
VectorValues optimizeGradientSearch(const GaussianBayesTree& bayesTree);

/** In-place version of optimizeGradientSearch requiring pre-allocated VectorValues \c x */
void optimizeGradientSearchInPlace(const GaussianBayesTree& bayesTree, VectorValues& grad);

/**
 * Compute the gradient of the energy function,
 * \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
 * centered around \f$ x = x_0 \f$.
 * The gradient is \f$ R^T(Rx-d) \f$.
 * @param bayesTree The Gaussian Bayes Tree $(R,d)$
 * @param x0 The center about which to compute the gradient
 * @return The gradient as a VectorValues
 */
VectorValues gradient(const GaussianBayesTree& bayesTree, const VectorValues& x0);

/**
 * Compute the gradient of the energy function,
 * \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
 * centered around zero.
 * The gradient about zero is \f$ -R^T d \f$.  See also gradient(const GaussianBayesNet&, const VectorValues&).
 * @param bayesTree The Gaussian Bayes Tree $(R,d)$
 * @param [output] g A VectorValues to store the gradient, which must be preallocated, see allocateVectorValues
 * @return The gradient as a VectorValues
 */
void gradientAtZero(const GaussianBayesTree& bayesTree, VectorValues& g);

}

#include <gtsam/linear/GaussianBayesTree-inl.h>

