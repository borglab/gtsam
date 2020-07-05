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

#include "FunctorizedFactor.h"

namespace gtsam {

/**
 * Factor for BASIS evaluation.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 */
template <class BASIS>
using PredictFactor = FunctorizedFactor<typename BASIS::EvaluationFunctor>;

/**
 * Factor for BASIS derivative evaluation.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 */
template <class BASIS>
using DerivativeFactor = FunctorizedFactor<typename BASIS::DerivativeFunctor>;

/**
 * Prior factor for BASIS vector evaluation on Matrix of size (M, N_ORDER).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param M: Size of the evaluated state vector.
 */
template <class BASIS, int M>
using VectorPrior =
    FunctorizedFactor<typename BASIS::template VectorEvaluationFunctor<M>>;

/**
 * Prior factor for BASIS Manifold evaluation on type T.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param T: Object type which is synthesized by the functor.
 */
template <typename BASIS, typename T>
using TypePrior =
    FunctorizedFactor<typename BASIS::template ManifoldEvaluationFunctor<T>>;

/**
 * Prior factor for BASIS component evaluation on Matrix of size (M, N_ORDER).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param P: Size of the component.
 */
template <typename BASIS, size_t P>
using ComponentPrior =
    FunctorizedFactor<typename BASIS::template ComponentEvaluationFunctor<P>>;

/**
 * Prior factor for BASIS vector derivative on Matrix of size (M, N_ORDER).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param M: Size of the evaluated state vector derivative.
 */
template <typename BASIS, int M>
using VectorDerivativePrior =
    FunctorizedFactor<typename BASIS::template VectorDerivativeFunctor<M>>;

/**
 * Prior factor for BASIS component derivative on Matrix of size (M, N_ORDER).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param P: Size of the component derivative.
 */
template <typename BASIS, int P>
using ComponentDerivativePrior =
    FunctorizedFactor<typename BASIS::template ComponentDerivativeFunctor<P>>;

}  // namespace gtsam
