/**
 * @file ConstraintOptimizer.h
 * @brief Utilities and classes required for solving Constrained Problems
 * @author Alex Cunningham
 */

/**
 * IMPORTANT NOTE: This is an EXPERIMENTAL system that is not ready for use!
 * DO NOT USE if you actually wanted to accomplish something
 *
 * REQUIRES --enable-ldl flag to be set for this class to work, not compiled otherwise
 */

#pragma once

#include <boost/optional.hpp>
#include <Matrix.h>

namespace gtsam {

	/**
	 * BFGS Hessian estimator
	 * Maintains an estimate for a hessian matrix using
	 * only derivatives and the step
	 */
	class BFGSEstimator {
	protected:
		size_t n_; // dimension of square matrix
		Matrix B_; // current estimate
		Vector prev_dfx_; // previous gradient value
	public:

		/**
		 * Creates an estimator of a particular dimension
		 */
		BFGSEstimator(size_t n) : n_(n), B_(eye(n,n)) {}

		~BFGSEstimator() {}

		/**
		 * Direct vector interface - useful for small problems
		 *
		 * Update will set the previous gradient and update the
		 * estimate for the Hessian.  When specified without
		 * the step, this is the initialization phase, and will not
		 * change the Hessian estimate
		 * @param df is the gradient of the function
		 * @param step is the step vector applied at the last iteration
		 */
		void update(const Vector& df, const boost::optional<Vector&> step = boost::none);

		// access functions
		const Matrix& getB() const { return B_; }
		size_t dim() const { return n_; }

	};


	/**
	 * Basic function that uses LDL factorization to solve a
	 * KKT system (state and lagrange multipliers) of the form:
	 * Gd=b, where
	 * G = [B  A]  d = [ x ] b = - [g]
	 *     [A' 0]      [lam]       [h]
	 * B = Hessian of Lagragian function
	 * A = Gradient of constraints
	 * x = state
	 * lam = vector of lagrange mulipliers
	 * g = gradient of f(x) evaluated a point
	 * h = current value of c(x)
	 *
	 * @return pair of state and lambas
	 */
	std::pair<Vector, Vector> solveCQP(const Matrix& B, const Matrix& A,
									   const Vector& g, const Vector& h);

	/**
	 * Simple line search function using an externally specified
	 * penalty function
	 */
	Vector linesearch(const Vector& x, const Vector& delta,
			double (*penalty)(const Vector&), size_t maxIt = 10);

} // \namespace gtsam

