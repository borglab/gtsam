/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BasisFactors.h
 *  @brief Factor definitions for various Basis functors.
 *  @author Varun Agrawal
 *  @date July 4, 2020
 **/

#pragma once

#include <gtsam/basis/Basis.h>
#include <gtsam/nonlinear/FunctorizedFactor.h>

namespace gtsam {

/**
 * @brief Factor for scalar BASIS evaluation.
 *
 * @tparam BASIS The basis class to use e.g. Chebyshev2
 */
template <class BASIS>
class GTSAM_EXPORT EvaluationFactor : public FunctorizedFactor<double, Vector> {
 private:
  using Base = FunctorizedFactor<double, Vector>;

 public:
  EvaluationFactor() {}

  /**
   * @brief Construct a new EvaluationFactor object
   *
   * @param key Symbol for value to optimize.
   * @param z The result of the functional evaluation.
   * @param model Noise model
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the polynomial.
   */
  EvaluationFactor(Key key, const double &z, const SharedNoiseModel &model,
                   const size_t N, double x)
      : Base(key, z, model, typename BASIS::EvaluationFunctor(N, x)) {}

  /**
   * @brief Construct a new EvaluationFactor object
   *
   * @param key Symbol for value to optimize.
   * @param z The result of the functional evaluation.
   * @param model Noise model
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the polynomial.
   * @param a Lower bound for the polynomial.
   * @param b Upper bound for the polynomial.
   */
  EvaluationFactor(Key key, const double &z, const SharedNoiseModel &model,
                   const size_t N, double x, double a, double b)
      : Base(key, z, model, typename BASIS::EvaluationFunctor(N, x, a, b)) {}

  virtual ~EvaluationFactor() {}
};

/**
 * Prior factor for BASIS vector evaluation on ParameterMatrix of size (M,N).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param M: Size of the evaluated state vector.
 */
template <class BASIS, int M>
class GTSAM_EXPORT VectorEvaluationFactor
    : public FunctorizedFactor<Vector, ParameterMatrix<M>> {
 private:
  using Base = FunctorizedFactor<Vector, ParameterMatrix<M>>;

 public:
  VectorEvaluationFactor() {}

  VectorEvaluationFactor(Key key, const Vector &z,
                         const SharedNoiseModel &model, const size_t N,
                         double x)
      : Base(key, z, model,
             typename BASIS::template VectorEvaluationFunctor<M>(N, x)) {}

  VectorEvaluationFactor(Key key, const Vector &z,
                         const SharedNoiseModel &model, const size_t N,
                         double x, double a, double b)
      : Base(key, z, model,
             typename BASIS::template VectorEvaluationFunctor<M>(N, x, a, b)) {}

  virtual ~VectorEvaluationFactor() {}
};

/**
 * Create a measurement on any scalar entry (chosen in the constructor) of a
 * P-dimensional vector computed by a basis parameterization.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param P: Size of the fixed-size vector.
 *
 * Example:
 *  VectorComponentFactor<BASIS, P> controlPrior(key, measured, model,
 *                                               N, i, t, a, b);
 *  where N is the degree and i is the component index.
 */
template <class BASIS, size_t P>
class GTSAM_EXPORT VectorComponentFactor
    : public FunctorizedFactor<double, ParameterMatrix<P>> {
 private:
  using Base = FunctorizedFactor<double, ParameterMatrix<P>>;

 public:
  VectorComponentFactor() {}

  VectorComponentFactor(Key key, const double &z, const SharedNoiseModel &model,
                        const size_t N, size_t i, double x)
      : Base(key, z, model,
             typename BASIS::template VectorComponentFunctor<P>(N, i, x)) {}

  VectorComponentFactor(Key key, const double &z, const SharedNoiseModel &model,
                        const size_t N, size_t i, double x, double a, double b)
      : Base(
            key, z, model,
            typename BASIS::template VectorComponentFunctor<P>(N, i, x, a, b)) {
  }

  virtual ~VectorComponentFactor() {}
};

/**
 * Prior factor for BASIS Manifold evaluation on type T.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param T: Object type which is synthesized by the functor.
 */
template <class BASIS, typename T>
class GTSAM_EXPORT ManifoldEvaluationFactor
    : public FunctorizedFactor<T, ParameterMatrix<traits<T>::dimension>> {
 private:
  using Base = FunctorizedFactor<T, ParameterMatrix<traits<T>::dimension>>;

 public:
  ManifoldEvaluationFactor() {}

  ManifoldEvaluationFactor(Key key, const T &z, const SharedNoiseModel &model,
                           const size_t N, double x)
      : Base(key, z, model,
             typename BASIS::template ManifoldEvaluationFunctor<T>(N, x)) {}

  ManifoldEvaluationFactor(Key key, const T &z, const SharedNoiseModel &model,
                           const size_t N, double x, double a, double b)
      : Base(
            key, z, model,
            typename BASIS::template ManifoldEvaluationFunctor<T>(N, x, a, b)) {
  }

  virtual ~ManifoldEvaluationFactor() {}
};

/**
 * Factor for scalar BASIS derivative evaluation.
 * @param BASIS: The basis class to use e.g. Chebyshev2
 */
template <class BASIS>
class GTSAM_EXPORT DerivativeFactor
    : public FunctorizedFactor<double, typename BASIS::Parameters> {
 private:
  using Base = FunctorizedFactor<double, typename BASIS::Parameters>;

 public:
  DerivativeFactor() {}

  DerivativeFactor(Key key, const double &z, const SharedNoiseModel &model,
                   const size_t N, double x)
      : Base(key, z, model, typename BASIS::DerivativeFunctor(N, x)) {}

  DerivativeFactor(Key key, const double &z, const SharedNoiseModel &model,
                   const size_t N, double x, double a, double b)
      : Base(key, z, model, typename BASIS::DerivativeFunctor(N, x, a, b)) {}

  virtual ~DerivativeFactor() {}
};

/**
 * Prior factor for BASIS vector derivative on ParameterMatrix of size (M,N).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param M: Size of the evaluated state vector derivative.
 */
template <class BASIS, int M>
class GTSAM_EXPORT VectorDerivativeFactor
    : public FunctorizedFactor<Vector, ParameterMatrix<M>> {
 private:
  using Base = FunctorizedFactor<Vector, ParameterMatrix<M>>;
  using Func = typename BASIS::template VectorDerivativeFunctor<M>;

 public:
  VectorDerivativeFactor() {}

  VectorDerivativeFactor(Key key, const Vector &z,
                         const SharedNoiseModel &model, const size_t N,
                         double x)
      : Base(key, z, model, Func(N, x)) {}

  VectorDerivativeFactor(Key key, const Vector &z,
                         const SharedNoiseModel &model, const size_t N,
                         double x, double a, double b)
      : Base(key, z, model, Func(N, x, a, b)) {}

  virtual ~VectorDerivativeFactor() {}
};

/**
 * Prior factor for BASIS component derivative on ParameterMatrix of size (M,N).
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param P: Size of the control component derivative.
 */
template <class BASIS, int P>
class GTSAM_EXPORT ComponentDerivativeFactor
    : public FunctorizedFactor<double, ParameterMatrix<P>> {
 private:
  using Base = FunctorizedFactor<double, ParameterMatrix<P>>;
  using Func = typename BASIS::template ComponentDerivativeFunctor<P>;

 public:
  ComponentDerivativeFactor() {}

  ComponentDerivativeFactor(Key key, const double &z,
                            const SharedNoiseModel &model, const size_t N,
                            size_t i, double x)
      : Base(key, z, model, Func(N, i, x)) {}

  ComponentDerivativeFactor(Key key, const double &z,
                            const SharedNoiseModel &model, const size_t N,
                            size_t i, double x, double a, double b)
      : Base(key, z, model, Func(N, i, x, a, b)) {}

  virtual ~ComponentDerivativeFactor() {}
};

}  // namespace gtsam
