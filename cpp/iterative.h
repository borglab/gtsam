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
	 * Method of conjugate gradients (CG) template
	 * "System" class S needs gradient(S,v), e=S*v, v=S^e
	 * "Vector" class V needs dot(v,v), -v, v+v, s*v
	 * "Vector" class E needs dot(v,v)
	 * @param Ab, the "system" that needs to be solved, examples below
	 * @param x is the initial estimate
	 * @param epsilon determines the convergence criterion: norm(g)<epsilon*norm(g0)
	 * @param maxIterations, if 0 will be set to |x|
	 * @param steepest flag, if true does steepest descent, not CG
	 * */
	template<class S, class V, class E>
	V conjugateGradients(const S& Ab, V x, bool verbose, double epsilon,
			size_t maxIterations, bool steepest = false);

	/**
	 * Method of steepest gradients, System version
	 */
	Vector steepestDescent(const System& Ab, const Vector& x, bool verbose =
			false, double epsilon = 1e-3, size_t maxIterations = 0);

	/**
	 * Method of steepest gradients, Matrix version
	 */
	Vector steepestDescent(const Matrix& A, const Vector& b, const Vector& x,
			bool verbose = false, double epsilon = 1e-3, size_t maxIterations = 0);

	/**
	 * Method of steepest gradients, Gaussian Factor Graph version
	 * */
	VectorConfig steepestDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, bool verbose = false, double epsilon = 1e-3,
			size_t maxIterations = 0);

	/**
	 * Method of conjugate gradients (CG), System version
	 */
	Vector conjugateGradientDescent(const System& Ab, const Vector& x,
			bool verbose = false, double epsilon = 1e-3, size_t maxIterations = 0);

	/**
	 * Method of conjugate gradients (CG), Matrix version
	 */
	Vector conjugateGradientDescent(const Matrix& A, const Vector& b,
			const Vector& x, bool verbose = false, double epsilon = 1e-3,
			size_t maxIterations = 0);

	/**
	 * Method of conjugate gradients (CG), Gaussian Factor Graph version
	 * */
	VectorConfig conjugateGradientDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, bool verbose = false, double epsilon = 1e-3,
			size_t maxIterations = 0);

} // namespace gtsam
