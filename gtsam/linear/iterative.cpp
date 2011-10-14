/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file iterative.cpp
 * @brief Iterative methods, implementation
 * @author Frank Dellaert
 * @date Dec 28, 2009
 */
#include <iostream>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/IterativeOptimizationParameters.h>
#include <gtsam/linear/iterative-inl.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	void System::print (const string& s) const {
		cout << s << endl;
		gtsam::print(A_, "A");
		gtsam::print(b_, "b");
	}

	/* ************************************************************************* */

	Vector steepestDescent(const System& Ab, const Vector& x, const IterativeOptimizationParameters & parameters) {
		return conjugateGradients<System, Vector, Vector> (Ab, x, parameters, true);
	}

	Vector conjugateGradientDescent(const System& Ab, const Vector& x, const IterativeOptimizationParameters & parameters) {
		return conjugateGradients<System, Vector, Vector> (Ab, x, parameters);
	}

	/* ************************************************************************* */
	Vector steepestDescent(const Matrix& A, const Vector& b, const Vector& x, const IterativeOptimizationParameters & parameters) {
		System Ab(A, b);
		return conjugateGradients<System, Vector, Vector> (Ab, x, parameters, true);
	}

	Vector conjugateGradientDescent(const Matrix& A, const Vector& b, const Vector& x, const IterativeOptimizationParameters & parameters) {
		System Ab(A, b);
		return conjugateGradients<System, Vector, Vector> (Ab, x, parameters);
	}

	/* ************************************************************************* */
	VectorValues steepestDescent(const FactorGraph<JacobianFactor>& fg, const VectorValues& x, const IterativeOptimizationParameters & parameters) {
		return conjugateGradients<FactorGraph<JacobianFactor>, VectorValues, Errors> (fg, x, parameters, true);
	}

	VectorValues conjugateGradientDescent(const FactorGraph<JacobianFactor>& fg, const VectorValues& x, const IterativeOptimizationParameters & parameters) {
		return conjugateGradients<FactorGraph<JacobianFactor>, VectorValues, Errors> (fg, x, parameters);
	}

	/* ************************************************************************* */

} // namespace gtsam
