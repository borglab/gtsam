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

#include <gtsam/base/types.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/inference/BayesNet.h>

namespace gtsam {

	/** A Bayes net made from linear-Gaussian densities */
	typedef BayesNet<GaussianConditional> GaussianBayesNet;

	/** Create a scalar Gaussian */
	GaussianBayesNet scalarGaussian(Index key, double mu=0.0, double sigma=1.0);

	/** Create a simple Gaussian on a single multivariate variable */
	GaussianBayesNet simpleGaussian(Index key, const Vector& mu, double sigma=1.0);

	/**
	 * Add a conditional node with one parent
	 * |Rx+Sy-d|
	 */
	void push_front(GaussianBayesNet& bn, Index key, Vector d, Matrix R,
			Index name1, Matrix S, Vector sigmas);

	/**
	 * Add a conditional node with two parents
	 * |Rx+Sy+Tz-d|
	 */
	void push_front(GaussianBayesNet& bn, Index key, Vector d, Matrix R,
			Index name1, Matrix S, Index name2, Matrix T, Vector sigmas);

	/**
	 * Allocate a VectorValues for the variables in a BayesNet
	 */
	boost::shared_ptr<VectorValues> allocateVectorValues(const GaussianBayesNet& bn);

	/**
	 * optimize, i.e. return x = inv(R)*d
	 */
	VectorValues optimize(const GaussianBayesNet&);

	/**
	 * shared pointer version
	 */
	boost::shared_ptr<VectorValues> optimize_(const GaussianBayesNet& bn);

	/**
	 * Backsubstitute
	 * (R*x)./sigmas = y by solving x=inv(R)*(y.*sigmas)
	 * @param y is the RHS of the system
	 */
	VectorValues backSubstitute(const GaussianBayesNet& bn, const VectorValues& y);

	/**
	 * Backsubstitute in place, y starts as RHS and is replaced with solution
	 */
	void backSubstituteInPlace(const GaussianBayesNet& bn, VectorValues& y);

	/**
	 * Transpose Backsubstitute
	 * gy=inv(L)*gx by solving L*gy=gx.
	 * gy=inv(R'*inv(Sigma))*gx
	 * gz'*R'=gx', gy = gz.*sigmas
	 */
	VectorValues backSubstituteTranspose(const GaussianBayesNet& bn, const VectorValues& gx);

	/**
	 * Return (dense) upper-triangular matrix representation
	 * NOTE: if this is the result of elimination with LDL, the matrix will
	 * not necessarily be upper triangular due to column permutations
	 */
	std::pair<Matrix, Vector> matrix(const GaussianBayesNet&);

  /**
   * Return RHS d as a VectorValues
   * Such that backSubstitute(bn,d) = optimize(bn)
   */
  VectorValues rhs(const GaussianBayesNet&);

  /**
   * Computes the determinant of a GassianBayesNet
   * A GaussianBayesNet is an upper triangular matrix and for an upper triangular matrix
   * determinant is the product of the diagonal elements. Instead of actually multiplying
   * we add the logarithms of the diagonal elements and take the exponent at the end
   * because this is more numerically stable.
   * @param bayesNet The input GaussianBayesNet
   * @return The determinant
   */
  double determinant(const GaussianBayesNet& bayesNet);

  /**
   * Compute the gradient of the energy function,
   * \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
   * centered around \f$ x = x_0 \f$.
   * The gradient is \f$ R^T(Rx-d) \f$.
   * @param bayesNet The Gaussian Bayes net $(R,d)$
   * @param x0 The center about which to compute the gradient
   * @return The gradient as a VectorValues
   */
  VectorValues gradient(const GaussianBayesNet& bayesNet, const VectorValues& x0);

  /**
   * Compute the gradient of the energy function,
   * \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} R x - d \right\Vert^2 \f$,
   * centered around zero.
   * The gradient about zero is \f$ -R^T d \f$.  See also gradient(const GaussianBayesNet&, const VectorValues&).
   * @param bayesNet The Gaussian Bayes net $(R,d)$
   * @param [output] g A VectorValues to store the gradient, which must be preallocated, see allocateVectorValues
   * @return The gradient as a VectorValues
   */
  void gradientAtZero(const GaussianBayesNet& bayesNet, VectorValues& g);

} /// namespace gtsam
