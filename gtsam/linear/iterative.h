/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file iterative.h
 * @brief Iterative methods, implementation
 * @author Frank Dellaert
 * @date Dec 28, 2009
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/ConjugateGradientSolver.h>

namespace gtsam {

	/**
	 * Method of conjugate gradients (CG) template
	 * "System" class S needs gradient(S,v), e=S*v, v=S^e
	 * "Vector" class V needs dot(v,v), -v, v+v, s*v
	 * "Vector" class E needs dot(v,v)
	 * @param Ab, the "system" that needs to be solved, examples below
	 * @param x is the initial estimate
	 * @param steepest flag, if true does steepest descent, not CG
	 * */
  template<class S, class V, class E>
  V conjugateGradients(const S& Ab, V x,
      const ConjugateGradientParameters &parameters, bool steepest = false);

	/**
	 * Helper class encapsulating the combined system |Ax-b_|^2
	 * Needed to run Conjugate Gradients on matrices
	 * */
	class System {

	private:
		const Matrix& A_;
		const Vector& b_;

	public:

		System(const Matrix& A, const Vector& b) :
			A_(A), b_(b) {
		}

		/** Access A matrix */
		const Matrix& A() const { return A_; }

		/** Access b vector */
		const Vector& b() const { return b_; }

		/** Apply operator A'*e */
		Vector operator^(const Vector& e) const {
			return A_ ^ e;
		}

		/**
		 * Print with optional string
		 */
		void print (const std::string& s = "System") const;
	};

  /** gradient of objective function 0.5*|Ax-b_|^2 at x = A_'*(Ax-b_) */
  inline Vector gradient(const System& system, const Vector& x) {
    return system.A() ^ (system.A() * x - system.b());
  }

  /** Apply operator A */
  inline Vector operator*(const System& system, const Vector& x) {
    return system.A() * x;
  }

  /** Apply operator A in place */
  inline void multiplyInPlace(const System& system, const Vector& x, Vector& e) {
    e = system.A() * x;
  }

  /** x += alpha* A'*e */
  inline void transposeMultiplyAdd(const System& system, double alpha, const Vector& e, Vector& x) {
    transposeMultiplyAdd(alpha,system.A(),e,x);
  }

	/**
	 * Method of steepest gradients, System version
	 */
	Vector steepestDescent(
			const System& Ab,
			const Vector& x,
			const IterativeOptimizationParameters & parameters);

	/**
	 * Method of conjugate gradients (CG), System version
	 */
	Vector conjugateGradientDescent(
			const System& Ab,
			const Vector& x,
			const ConjugateGradientParameters & parameters);

	/** convenience calls using matrices, will create System class internally: */

	/**
	 * Method of steepest gradients, Matrix version
	 */
	Vector steepestDescent(
			const Matrix& A,
			const Vector& b,
			const Vector& x,
			const ConjugateGradientParameters & parameters);

	/**
	 * Method of conjugate gradients (CG), Matrix version
	 */
	Vector conjugateGradientDescent(
			const Matrix& A,
			const Vector& b,
			const Vector& x,
			const ConjugateGradientParameters & parameters);

	/**
	 * Method of steepest gradients, Gaussian Factor Graph version
	 */
	VectorValues steepestDescent(
			const GaussianFactorGraph& fg,
			const VectorValues& x,
			const ConjugateGradientParameters & parameters);

	/**
	 * Method of conjugate gradients (CG), Gaussian Factor Graph version
	 */
	VectorValues conjugateGradientDescent(
			const GaussianFactorGraph& fg,
			const VectorValues& x,
			const ConjugateGradientParameters & parameters);


} // namespace gtsam

#include <gtsam/linear/iterative-inl.h>

