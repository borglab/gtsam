/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   GaussianBayesNet
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/global_includes.h>
#include <gtsam/linear/GaussianConditionalOrdered.h>
#include <gtsam/inference/BayesNetOrdered.h>

namespace gtsam {

  /** A Bayes net made from linear-Gaussian densities */
  typedef BayesNetOrdered<GaussianConditionalOrdered> GaussianBayesNetOrdered;

  /** Create a scalar Gaussian */
  GTSAM_EXPORT GaussianBayesNetOrdered scalarGaussian(Index key, double mu=0.0, double sigma=1.0);

  /** Create a simple Gaussian on a single multivariate variable */
  GTSAM_EXPORT GaussianBayesNetOrdered simpleGaussian(Index key, const Vector& mu, double sigma=1.0);

  /**
   * Add a conditional node with one parent
   * |Rx+Sy-d|
   */
  GTSAM_EXPORT void push_front(GaussianBayesNetOrdered& bn, Index key, Vector d, Matrix R,
      Index name1, Matrix S, Vector sigmas);

  /**
   * Add a conditional node with two parents
   * |Rx+Sy+Tz-d|
   */
  GTSAM_EXPORT void push_front(GaussianBayesNetOrdered& bn, Index key, Vector d, Matrix R,
      Index name1, Matrix S, Index name2, Matrix T, Vector sigmas);

  /**
   * Allocate a VectorValues for the variables in a BayesNet
   */
  GTSAM_EXPORT boost::shared_ptr<VectorValuesOrdered> allocateVectorValues(const GaussianBayesNetOrdered& bn);

  /**
   * Solve the GaussianBayesNet, i.e. return \f$ x = R^{-1}*d \f$, computed by
   * back-substitution.
   */
  GTSAM_EXPORT VectorValuesOrdered optimize(const GaussianBayesNetOrdered& bn);

  /**
   * Solve the GaussianBayesNet, i.e. return \f$ x = R^{-1}*d \f$, computed by
   * back-substitution, writes the solution \f$ x \f$ into a pre-allocated
   * VectorValues.  You can use allocateVectorValues(const GaussianBayesNet&)
   * allocate it.  See also optimize(const GaussianBayesNet&), which does not
   * require pre-allocation.
   */
  GTSAM_EXPORT void optimizeInPlace(const GaussianBayesNetOrdered& bn, VectorValuesOrdered& x);

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
   *
   * @param bn The GaussianBayesNet on which to perform this computation
   * @return The resulting \f$ \delta x \f$ as described above
   */
  GTSAM_EXPORT VectorValuesOrdered optimizeGradientSearch(const GaussianBayesNetOrdered& bn);

  /** In-place version of optimizeGradientSearch(const GaussianBayesNet&) requiring pre-allocated VectorValues \c grad
   *
   * @param bn The GaussianBayesNet on which to perform this computation
   * @param [out] grad The resulting \f$ \delta x \f$ as described in optimizeGradientSearch(const GaussianBayesNet&)
   * */
  GTSAM_EXPORT void optimizeGradientSearchInPlace(const GaussianBayesNetOrdered& bn, VectorValuesOrdered& grad);

  /**
   * Backsubstitute
   * gy=inv(R*inv(Sigma))*gx
   */
  GTSAM_EXPORT VectorValuesOrdered backSubstitute(const GaussianBayesNetOrdered& bn, const VectorValuesOrdered& gx);

  /**
   * Transpose Backsubstitute
   * gy=inv(L)*gx by solving L*gy=gx.
   * gy=inv(R'*inv(Sigma))*gx
   * gz'*R'=gx', gy = gz.*sigmas
   */
  GTSAM_EXPORT VectorValuesOrdered backSubstituteTranspose(const GaussianBayesNetOrdered& bn, const VectorValuesOrdered& gx);

  /**
   * Return (dense) upper-triangular matrix representation
   */
  GTSAM_EXPORT std::pair<Matrix, Vector> matrix(const GaussianBayesNetOrdered&);

  /**
   * Computes the determinant of a GassianBayesNet
   * A GaussianBayesNet is an upper triangular matrix and for an upper triangular matrix
   * determinant is the product of the diagonal elements. Instead of actually multiplying
   * we add the logarithms of the diagonal elements and take the exponent at the end
   * because this is more numerically stable.
   * @param bayesNet The input GaussianBayesNet
   * @return The determinant
   */
  GTSAM_EXPORT double determinant(const GaussianBayesNetOrdered& bayesNet);

  /**
   * Compute the gradient of the energy function,
   * \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
   * centered around \f$ x = x_0 \f$.
   * The gradient is \f$ R^T(Rx-d) \f$.
   * @param bayesNet The Gaussian Bayes net $(R,d)$
   * @param x0 The center about which to compute the gradient
   * @return The gradient as a VectorValues
   */
  GTSAM_EXPORT VectorValuesOrdered gradient(const GaussianBayesNetOrdered& bayesNet, const VectorValuesOrdered& x0);

  /**
   * Compute the gradient of the energy function,
   * \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
   * centered around zero.
   * The gradient about zero is \f$ -R^T d \f$.  See also gradient(const GaussianBayesNet&, const VectorValues&).
   * @param bayesNet The Gaussian Bayes net $(R,d)$
   * @param [output] g A VectorValues to store the gradient, which must be preallocated, see allocateVectorValues
   * @return The gradient as a VectorValues
   */
  GTSAM_EXPORT void gradientAtZero(const GaussianBayesNetOrdered& bayesNet, VectorValuesOrdered& g);

} /// namespace gtsam
