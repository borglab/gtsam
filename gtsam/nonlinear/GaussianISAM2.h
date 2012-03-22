/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianISAM
 * @brief   Full non-linear ISAM.
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace gtsam {

/**
 * @ingroup ISAM2
 * @brief The main ISAM2 class that is exposed to gtsam users, see ISAM2 for usage.
 *
 * This is a thin wrapper around an ISAM2 class templated on
 * GaussianConditional, and the values on which that GaussianISAM2 is
 * templated.
 *
 * @tparam VALUES The Values or TupleValues\Emph{N} that contains the
 * variables.
 * @tparam GRAPH The NonlinearFactorGraph structure to store factors.  Defaults to standard NonlinearFactorGraph<VALUES>
 */
template <class GRAPH = NonlinearFactorGraph>
class GaussianISAM2 : public ISAM2<GaussianConditional, GRAPH> {
  typedef ISAM2<GaussianConditional, GRAPH> Base;
public:

	/// @name Standard Constructors
	/// @{

  /** Create an empty ISAM2 instance */
  GaussianISAM2(const ISAM2Params& params) : ISAM2<GaussianConditional, GRAPH>(params) {}

  /** Create an empty ISAM2 instance using the default set of parameters (see ISAM2Params) */
  GaussianISAM2() : ISAM2<GaussianConditional, GRAPH>() {}

	/// @}
	/// @name Advanced Interface
	/// @{

  void cloneTo(boost::shared_ptr<GaussianISAM2>& newGaussianISAM2) const {
    boost::shared_ptr<Base> isam2 = boost::static_pointer_cast<Base>(newGaussianISAM2);
    Base::cloneTo(isam2);
  }

	/// @}

};

/** Get the linear delta for the ISAM2 object, unpermuted the delta returned by ISAM2::getDelta() */
template<class GRAPH>
VectorValues optimize(const ISAM2<GaussianConditional, GRAPH>& isam) {
  VectorValues delta = *allocateVectorValues(isam);
  internal::optimizeInPlace(isam.root(), delta);
  return delta;
}

/// Optimize the BayesTree, starting from the root.
/// @param replaced Needs to contain
/// all variables that are contained in the top of the Bayes tree that has been
/// redone.
/// @param delta The current solution, an offset from the linearization
/// point.
/// @param threshold The maximum change against the PREVIOUS delta for
/// non-replaced variables that can be ignored, ie. the old delta entry is kept
/// and recursive backsubstitution might eventually stop if none of the changed
/// variables are contained in the subtree.
/// @return The number of variables that were solved for
template<class CLIQUE>
int optimizeWildfire(const boost::shared_ptr<CLIQUE>& root,
    double threshold, const std::vector<bool>& replaced, Permuted<VectorValues>& delta);

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
template<class GRAPH>
VectorValues optimizeGradientSearch(const ISAM2<GaussianConditional, GRAPH>& isam);

/** In-place version of optimizeGradientSearch requiring pre-allocated VectorValues \c x */
template<class GRAPH>
void optimizeGradientSearchInPlace(const ISAM2<GaussianConditional, GRAPH>& isam, VectorValues& grad);

/// calculate the number of non-zero entries for the tree starting at clique (use root for complete matrix)
template<class CLIQUE>
int calculate_nnz(const boost::shared_ptr<CLIQUE>& clique);

/**
 * Compute the gradient of the energy function,
 * \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
 * centered around \f$ x = x_0 \f$.
 * The gradient is \f$ R^T(Rx-d) \f$.
 * This specialized version is used with ISAM2, where each clique stores its
 * gradient contribution.
 * @param bayesTree The Gaussian Bayes Tree $(R,d)$
 * @param x0 The center about which to compute the gradient
 * @return The gradient as a VectorValues
 */
VectorValues gradient(const BayesTree<GaussianConditional, ISAM2Clique<GaussianConditional> >& bayesTree, const VectorValues& x0);

/**
 * Compute the gradient of the energy function,
 * \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
 * centered around zero.
 * The gradient about zero is \f$ -R^T d \f$.  See also gradient(const GaussianBayesNet&, const VectorValues&).
 * This specialized version is used with ISAM2, where each clique stores its
 * gradient contribution.
 * @param bayesTree The Gaussian Bayes Tree $(R,d)$
 * @param [output] g A VectorValues to store the gradient, which must be preallocated, see allocateVectorValues
 * @return The gradient as a VectorValues
 */
void gradientAtZero(const BayesTree<GaussianConditional, ISAM2Clique<GaussianConditional> >& bayesTree, VectorValues& g);

}/// namespace gtsam

#include <gtsam/nonlinear/GaussianISAM2-inl.h>
