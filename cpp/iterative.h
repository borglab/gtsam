/*
 * iterative.h
 * @brief Iterative methods, implementation
 * @author Frank Dellaert
 * Created on: Dec 28, 2009
 */

#include "Matrix.h"
namespace gtsam {

	class GaussianFactorGraph;
	class VectorConfig;

	/** typedef for combined system |Ax-b|^2 */
	typedef std::pair<Matrix, Vector> System;

	/**
	 * In all calls below
	 * x is the initial estimate
	 * epsilon determines the convergence criterion: norm(g)<epsilon*norm(g0)
	 */

	/**
	 * Method of steepest gradients, System version
	 */
	Vector steepestDescent(const System& Ab, const Vector& x, double epsilon =
			1e-5, size_t maxIterations = 0);

	/**
	 * Method of steepest gradients, Matrix version
	 */
	Vector steepestDescent(const Matrix& A, const Vector& b, const Vector& x,
			double epsilon = 1e-5, size_t maxIterations = 0);

	/**
	 * Method of steepest gradients, Gaussian Factor Graph version
	 * */
	VectorConfig steepestDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, double epsilon = 1e-5, size_t maxIterations = 0);

	/**
	 * Method of conjugate gradients (CG), System version
	 */
	Vector conjugateGradientDescent(const System& Ab, const Vector& x,
			double epsilon = 1e-5, size_t maxIterations = 0);

	/**
	 * Method of conjugate gradients (CG), Matrix version
	 */
	Vector conjugateGradientDescent(const Matrix& A, const Vector& b,
			const Vector& x, double epsilon = 1e-5, size_t maxIterations = 0);

	/**
	 * Method of conjugate gradients (CG), Gaussian Factor Graph version
	 * */
	VectorConfig conjugateGradientDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, double epsilon = 1e-5, size_t maxIterations = 0);

} // namespace gtsam
