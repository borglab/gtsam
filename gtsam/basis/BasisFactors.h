/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BasisFactors.h
 *  @author Varun Agrawal
 **/

#pragma once

#include <gtsam/nonlinear/FunctorizedFactor.h>

namespace gtsam {

/**
 * Factor for BASIS evaluation.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 */
template <class BASIS>
using PredictFactor =
    FunctorizedFactor<typename BASIS::EvaluationFunctor::return_type,
                      typename BASIS::EvaluationFunctor::argument_type>;

/**
 * Factor for BASIS derivative evaluation.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 */
template <class BASIS>
using DerivativeFactor =
    FunctorizedFactor<typename BASIS::DerivativeFunctor::return_type,
                      typename BASIS::DerivativeFunctor::argument_type>;

/**
 * Prior factor for BASIS vector evaluation on Matrix of size (M, N_ORDER).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param M: Size of the evaluated state vector.
 */
template <class BASIS, int M>
using VectorPrior = FunctorizedFactor<
    typename BASIS::template VectorEvaluationFunctor<M>::return_type,
    typename BASIS::template VectorEvaluationFunctor<M>::argument_type>;

/**
 * Prior factor for BASIS Manifold evaluation on type T.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param T: Object type which is synthesized by the functor.
 */
template <typename BASIS, typename T>
using TypePrior = FunctorizedFactor<
    typename BASIS::template ManifoldEvaluationFunctor<T>::return_type,
    typename BASIS::template ManifoldEvaluationFunctor<T>::argument_type>;

/**
 * Prior factor for BASIS component evaluation on Matrix of size (M, N_ORDER).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param P: Size of the component.
 */
template <typename BASIS, size_t P>
using ComponentPrior = FunctorizedFactor<
    typename BASIS::template ComponentEvaluationFunctor<P>::return_type,
    typename BASIS::template ComponentEvaluationFunctor<P>::argument_type>;

/**
 * Prior factor for BASIS vector derivative on Matrix of size (M, N_ORDER).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param M: Size of the evaluated state vector derivative.
 */
template <typename BASIS, int M>
using VectorDerivativePrior = FunctorizedFactor<
    typename BASIS::template VectorDerivativeFunctor<M>::return_type,
    typename BASIS::template VectorDerivativeFunctor<M>::argument_type>;

/**
 * Prior factor for BASIS component derivative on Matrix of size (M, N_ORDER).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param P: Size of the component derivative.
 */
template <typename BASIS, int P>
using ComponentDerivativePrior = FunctorizedFactor<
    typename BASIS::template ComponentDerivativeFunctor<P>::return_type,
    typename BASIS::template ComponentDerivativeFunctor<P>::argument_type>;

}  // namespace gtsam
