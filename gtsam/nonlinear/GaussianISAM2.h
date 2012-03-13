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

/// optimize the BayesTree, starting from the root; "replaced" needs to contain
/// all variables that are contained in the top of the Bayes tree that has been
/// redone; "delta" is the current solution, an offset from the linearization
/// point; "threshold" is the maximum change against the PREVIOUS delta for
/// non-replaced variables that can be ignored, ie. the old delta entry is kept
/// and recursive backsubstitution might eventually stop if none of the changed
/// variables are contained in the subtree.
/// returns the number of variables that were solved for
template<class CLIQUE>
int optimizeWildfire(const boost::shared_ptr<CLIQUE>& root,
    double threshold, const std::vector<bool>& replaced, Permuted<VectorValues>& delta);

/// calculate the number of non-zero entries for the tree starting at clique (use root for complete matrix)
template<class CLIQUE>
int calculate_nnz(const boost::shared_ptr<CLIQUE>& clique);

/**
 * Compute the gradient of the energy function,
 * \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
 * centered around \f$ x = x_0 \f$.
 * The gradient is \f$ R^T(Rx-d) \f$.
 * @param bayesTree The Gaussian Bayes Tree $(R,d)$
 * @param x0 The center about which to compute the gradient
 * @return The gradient as a VectorValues
 */
VectorValues gradient(const BayesTree<GaussianConditional>& bayesTree, const VectorValues& x0);

/**
 * Compute the gradient of the energy function,
 * \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
 * centered around zero.
 * The gradient about zero is \f$ -R^T d \f$.  See also gradient(const GaussianBayesNet&, const VectorValues&).
 * @param bayesTree The Gaussian Bayes Tree $(R,d)$
 * @param [output] g A VectorValues to store the gradient, which must be preallocated, see allocateVectorValues
 * @return The gradient as a VectorValues
 */
void gradientAtZero(const BayesTree<GaussianConditional>& bayesTree, VectorValues& g);

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
