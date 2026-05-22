/** This header file provides a set of concepts that are useful for implementing
 * convex optimization algorithms. For more information, please consult the
 * excellent references:
 *
 * "Convex Optimization" by Boyd and Vandenberghe
 *
 * "Proximal Algorithms" by Parikh and Boyd
 *
 * Copyright (C) 2017 - 2022 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include "Optimization/Base/Concepts.h"

namespace Optimization {

namespace Convex {

/** Useful typedefs for implementing templated/parameterized convex optimization
 * methods.
 *
 * In the following definitions:
 *
 * - Variable is the type of the argument of the objective function (i.e., this
 * is the type of variable to be optimized).  As this represents an element of a
 * convex set (a subset of an affine space), this type must support addition,
 * subtraction, and scalar multiplication (implemented as operator +, -, and *,
 * respectively), as well as well as an inner product operation, implemented as
 * a method .dot()
 *
 * - Args is a user-definable variadic template parameter whose instances will
 * be passed into each of the functions required to run the optimization
 * algorithm (objective, quadratic model constructor, linear operators,
 * retractions, and acceptance function); this enables e.g. the use of
 * objectives accepting non-optimized parameters of arbitrary types, and/or the
 * use of arbitrary user-defined cache variables in order to store and reuse
 * intermediate results in different parts of the computation.
 */

/** An alias template for a function that accepts as input an argument of
 * type Variable, and returns the gradient grad_f(x) of a function f, evaluated
 * at x.  Note that here we represent the gradient as another element of type
 * Variable (i.e., we are exploiting the identification T_A(X) = A between an
 * affine space A and its tangent spaces). */
template <typename Variable, typename... Args>
using GradientOperator =
    std::function<Variable(const Variable &X, Args &... args)>;

/** An alias template for an inner product */
template <typename Variable, typename Scalar = double, typename... Args>
using InnerProduct =
    std::function<Scalar(const Variable &X, const Variable &Y, Args &... args)>;

/** An alias template for a linear operator A : X -> Y between two (possibly
 * distinct) linear spaces.
 */
template <typename VariableX, typename VariableY, typename... Args>
using LinearOperator =
    std::function<VariableY(const VariableX &X, Args &... args)>;

/** An alias template for a proximal operator of a function f: given a
 * Variable x and a scalar lambda > 0, this function computes and returns
 * the value of the scaled proximal operator prox_{lambda f}(x):
 *
 * prox_{lambda f}(x) := argmin_z [ f(z) + (1 / (2*lambda)) ||z - x||_2 ]
 */
template <typename Variable, typename Scalar = double, typename... Args>
using ProximalOperator =
    std::function<Variable(const Variable &X, Scalar lambda, Args &... args)>;

} // namespace Convex
} // namespace Optimization
