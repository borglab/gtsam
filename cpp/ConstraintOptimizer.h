/**
 * @file ConstraintOptimizer.h
 * @brief Utilities and classes required for solving Constrained Problems
 * @author Alex Cunningham
 */

#pragma once

#include <Matrix.h>

namespace gtsam {

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

} // \namespace gtsam


