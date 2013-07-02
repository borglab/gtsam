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

#include <gtsam/inference/BayesTreeUnordered.h>
#include <gtsam/inference/BayesTreeCliqueBaseUnordered.h>

namespace gtsam {

  // Forward declarations
  class GaussianFactorGraphUnordered;
  class GaussianBayesNetUnordered;
  class GaussianConditionalUnordered;
  class VectorValuesUnordered;

  /* ************************************************************************* */
  /** A clique in a GaussianBayesTree */
  class GTSAM_EXPORT GaussianBayesTreeCliqueUnordered :
    public BayesTreeCliqueBaseUnordered<GaussianBayesTreeCliqueUnordered, GaussianFactorGraphUnordered, GaussianBayesNetUnordered>
  {
  public:
    typedef GaussianBayesTreeCliqueUnordered This;
    typedef BayesTreeCliqueBaseUnordered<GaussianBayesTreeCliqueUnordered, GaussianFactorGraphUnordered, GaussianBayesNetUnordered> Base;
    typedef boost::shared_ptr<This> shared_ptr;
    typedef boost::weak_ptr<This> weak_ptr;
    GaussianBayesTreeCliqueUnordered() {}
    GaussianBayesTreeCliqueUnordered(const boost::shared_ptr<GaussianConditionalUnordered>& conditional) : Base(conditional) {}
  };

  /* ************************************************************************* */
  /** A Bayes tree representing a Gaussian density */
  class GTSAM_EXPORT GaussianBayesTreeUnordered :
    public BayesTreeUnordered<GaussianBayesTreeCliqueUnordered>
  {
  private:
    typedef BayesTreeUnordered<GaussianBayesTreeCliqueUnordered> Base;

  public:
    typedef GaussianBayesTreeUnordered This;
    typedef boost::shared_ptr<This> shared_ptr;

    /** Check equality */
    bool equals(const This& other, double tol = 1e-9) const;

    /** Recursively optimize the BayesTree to produce a vector solution. */
    VectorValuesUnordered optimize() const;

    /** Recursively optimize the BayesTree to produce a vector solution (same as optimize()), but
     *  write the solution in place in \c result. */
    //void optimizeInPlace(VectorValuesUnordered& result) const;

    /**
     * Optimize along the gradient direction, with a closed-form computation to perform the line
     * search.  The gradient is computed about \f$ \delta x=0 \f$.
     *
     * This function returns \f$ \delta x \f$ that minimizes a reparametrized problem.  The error
     * function of a GaussianBayesNet is
     *
     * \f[ f(\delta x) = \frac{1}{2} |R \delta x - d|^2 = \frac{1}{2}d^T d - d^T R \delta x +
     * \frac{1}{2} \delta x^T R^T R \delta x \f]
     *
     * with gradient and Hessian
     *
     * \f[ g(\delta x) = R^T(R\delta x - d), \qquad G(\delta x) = R^T R. \f]
     *
     * This function performs the line search in the direction of the gradient evaluated at \f$ g =
     * g(\delta x = 0) \f$ with step size \f$ \alpha \f$ that minimizes \f$ f(\delta x = \alpha g)
     * \f$:
     *
     * \f[ f(\alpha) = \frac{1}{2} d^T d + g^T \delta x + \frac{1}{2} \alpha^2 g^T G g \f]
     *
     * Optimizing by setting the derivative to zero yields \f$ \hat \alpha = (-g^T g) / (g^T G g)
     * \f$.  For efficiency, this function evaluates the denominator without computing the Hessian
     * \f$ G \f$, returning
     *
     * \f[ \delta x = \hat\alpha g = \frac{-g^T g}{(R g)^T(R g)} \f] */
    VectorValuesUnordered optimizeGradientSearch() const;

    /** In-place version of optimizeGradientSearch requiring pre-allocated VectorValues \c x */
    void optimizeGradientSearchInPlace(VectorValuesUnordered& grad) const;

    /** Compute the gradient of the energy function, \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} R x -
     * d \right\Vert^2 \f$, centered around \f$ x = x_0 \f$. The gradient is \f$ R^T(Rx-d) \f$.
     * 
     * @param x0 The center about which to compute the gradient
     * @return The gradient as a VectorValues */
    VectorValuesUnordered gradient(const VectorValuesUnordered& x0) const;

    /** Compute the gradient of the energy function, \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} R x - d
     * \right\Vert^2 \f$, centered around zero. The gradient about zero is \f$ -R^T d \f$.  See also
     * gradient(const GaussianBayesNet&, const VectorValues&).
     * 
     * @param [output] g A VectorValues to store the gradient, which must be preallocated, see
     *        allocateVectorValues */
    void gradientAtZeroInPlace(VectorValuesUnordered& g) const;

    /** Computes the determinant of a GassianBayesTree, as if the Bayes tree is reorganized into a
     * matrix. A GassianBayesTree is equivalent to an upper triangular matrix, and for an upper
     * triangular matrix determinant is the product of the diagonal elements. Instead of actually
     * multiplying we add the logarithms of the diagonal elements and take the exponent at the end
     * because this is more numerically stable. */
    double determinant() const;

    /** Computes the determinant of a GassianBayesTree, as if the Bayes tree is reorganized into a
     * matrix. A GassianBayesTree is equivalent to an upper triangular matrix, and for an upper
     * triangular matrix determinant is the product of the diagonal elements. Instead of actually
     * multiplying we add the logarithms of the diagonal elements and take the exponent at the end
     * because this is more numerically stable. */
    double logDeterminant() const;

  };

}
