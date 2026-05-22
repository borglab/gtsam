/** This header file provides a set of concepts that are useful for
 * implementing linear-algebraic algorithms.
 *
 * Copyright (C) 2022 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include <functional>

namespace Optimization {
namespace LinearAlgebra {

/** An alias template for a linear operator T : X -> Y between linear spaces
 * whose elements are of type X and Y, respectively */
template <typename X, typename Y, typename... Args>
using LinearOperator = std::function<Y(const X &x, Args &... args)>;

/** An alias template for a symmetric linear operator T : X -> X */
template <typename X, typename... Args>
using SymmetricLinearOperator = LinearOperator<X, X, Args...>;

/** An alias template for an inner product on Vectors*/
template <typename Vector, typename Scalar = double, typename... Args>
using InnerProduct =
    std::function<Scalar(const Vector &X, const Vector &Y, Args &...)>;

} // namespace LinearAlgebra
} // namespace Optimization
