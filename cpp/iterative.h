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

	typedef std::pair<Matrix, Vector> System;

	/**
	 * Method of conjugate gradients (CG), System version
	 */
	Vector conjugateGradientDescent(const System& Ab, const Vector& x,
			double threshold = 1e-9);

	/**
	 * Method of conjugate gradients (CG), Matrix version
	 */
	Vector conjugateGradientDescent(const Matrix& A, const Vector& b,
			const Vector& x, double threshold = 1e-9);

	/**
	 * Method of conjugate gradients (CG), Gaussian Factor Graph version
	 * */
	VectorConfig conjugateGradientDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, double threshold = 1e-9);

} // namespace gtsam
