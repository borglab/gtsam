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
 * @brief Factor for enforcing the scalar value of the polynomial BASIS
 * representation at `x` is the same as the measurement `z` when using a
 * pseudo-spectral parameterization.
 *
 * @tparam BASIS The basis class to use e.g. Chebyshev2
 *
 * Example, degree 8 Chebyshev polynomial measured at x=0.5:
 *  EvaluationFactor<Chebyshev2> factor(key, measured, model, 8, 0.5);
 *
 * @ingroup basis
 */
template <class BASIS>
class EvaluationFactor : public FunctorizedFactor<double, Vector> {
 private:
  using Base = FunctorizedFactor<double, Vector>;

 public:
  EvaluationFactor() {}

  /**
   * @brief Construct a new EvaluationFactor object
   *
   * @param key Symbol for value to optimize.
   * @param z The measurement value.
   * @param model Noise model
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the polynomial.
   */
  EvaluationFactor(Key key, double z, const SharedNoiseModel &model,
                   const size_t N, double x)
      : Base(key, z, model, typename BASIS::EvaluationFunctor(N, x)) {}

  /**
   * @brief Construct a new EvaluationFactor object
   *
   * @param key Symbol for value to optimize.
   * @param z The measurement value.
   * @param model Noise model
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the polynomial.
   * @param a Lower bound for the polynomial.
   * @param b Upper bound for the polynomial.
   */
  EvaluationFactor(Key key, double z, const SharedNoiseModel &model,
                   const size_t N, double x, double a, double b)
      : Base(key, z, model, typename BASIS::EvaluationFunctor(N, x, a, b)) {}

  virtual ~EvaluationFactor() {}
};

/**
 * Unary factor for enforcing BASIS polynomial evaluation on a ParameterMatrix
 * of size (M, N) is equal to a vector-valued measurement at the same point,
 when
 * using a pseudo-spectral parameterization.
 *
 * This factors tries to enforce the basis function evaluation `f(x;p)` to take
 * on the value `z` at location `x`, providing a gradient on the parameters p.
 * In a probabilistic estimation context, `z` is known as a measurement, and the
 * parameterized basis function can be seen as a
 * measurement prediction function.
 *
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param M: Size of the evaluated state vector.
 *
 * @ingroup basis
 */
template <class BASIS, int M>
class VectorEvaluationFactor
    : public FunctorizedFactor<Vector, ParameterMatrix> {
 private:
  using Base = FunctorizedFactor<Vector, ParameterMatrix>;

 public:
  VectorEvaluationFactor() {}

  /**
   * @brief Construct a new VectorEvaluationFactor object.
   *
   * @param key The key to the ParameterMatrix object used to represent the
   * polynomial.
   * @param z The measurement value.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the basis polynomial.
   */
  VectorEvaluationFactor(Key key, const Vector &z,
                         const SharedNoiseModel &model, const size_t N,
                         double x)
      : Base(key, z, model,
             typename BASIS::template VectorEvaluationFunctor<M>(N, x)) {}

  /**
   * @brief Construct a new VectorEvaluationFactor object.
   *
   * @param key The key to the ParameterMatrix object used to represent the
   * polynomial.
   * @param z The measurement value.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the basis polynomial.
   * @param a Lower bound for the polynomial.
   * @param b Upper bound for the polynomial.
   */
  VectorEvaluationFactor(Key key, const Vector &z,
                         const SharedNoiseModel &model, const size_t N,
                         double x, double a, double b)
      : Base(key, z, model,
             typename BASIS::template VectorEvaluationFunctor<M>(N, x, a, b)) {}

  virtual ~VectorEvaluationFactor() {}
};

/**
 * Unary factor for enforcing BASIS polynomial evaluation on a ParameterMatrix
 * of size (P, N) is equal to specified measurement at the same point, when
 * using a pseudo-spectral parameterization.
 *
 * This factor is similar to `VectorEvaluationFactor` with the key difference
 * being that it only enforces the constraint for a single scalar in the vector,
 * indexed by `i`.
 *
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param P: Size of the fixed-size vector.
 *
 * Example:
 *  VectorComponentFactor<BASIS, P> controlPrior(key, measured, model,
 *                                               N, i, t, a, b);
 *  where N is the degree and i is the component index.
 *
 * @ingroup basis
 */
// TODO(Varun) remove template P
template <class BASIS, size_t P>
class VectorComponentFactor
    : public FunctorizedFactor<double, ParameterMatrix> {
 private:
  using Base = FunctorizedFactor<double, ParameterMatrix>;

 public:
  VectorComponentFactor() {}

  /**
   * @brief Construct a new VectorComponentFactor object.
   *
   * @param key The key to the ParameterMatrix object used to represent the
   * polynomial.
   * @param z The scalar value at a specified index `i` of the full measurement
   * vector.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param i The index for the evaluated vector to give us the desired scalar
   * value.
   * @param x The point at which to evaluate the basis polynomial.
   */
  VectorComponentFactor(Key key, const double &z, const SharedNoiseModel &model,
                        const size_t N, size_t i, double x)
      : Base(key, z, model,
             typename BASIS::template VectorComponentFunctor<P>(N, i, x)) {}

  /**
   * @brief Construct a new VectorComponentFactor object.
   *
   * @param key The key to the ParameterMatrix object used to represent the
   * polynomial.
   * @param z The scalar value at a specified index `i` of the full measurement
   * vector.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param i The index for the evaluated vector to give us the desired scalar
   * value.
   * @param x The point at which to evaluate 0the basis polynomial.
   * @param a Lower bound for the polynomial.
   * @param b Upper bound for the polynomial.
   */
  VectorComponentFactor(Key key, const double &z, const SharedNoiseModel &model,
                        const size_t N, size_t i, double x, double a, double b)
      : Base(
            key, z, model,
            typename BASIS::template VectorComponentFunctor<P>(N, i, x, a, b)) {
  }

  virtual ~VectorComponentFactor() {}
};

/**
 * For a measurement value of type T i.e. `T z = g(x)`, this unary
 * factor enforces that the polynomial basis, when evaluated at `x`, gives us
 * the same `z`, i.e. `T z = g(x) = f(x;p)`.
 *
 * This is done via computations on the tangent space of the
 * manifold of T.
 *
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param T: Object type which is synthesized by the provided functor.
 *
 * Example:
 *  ManifoldEvaluationFactor<Chebyshev2, Rot3> rotationFactor(key, measurement,
 * model, N, x, a, b);
 *
 * where `x` is the value (e.g. timestep) at which the rotation was evaluated.
 */
template <class BASIS, typename T>
class ManifoldEvaluationFactor : public FunctorizedFactor<T, ParameterMatrix> {
 private:
  using Base = FunctorizedFactor<T, ParameterMatrix>;

 public:
  ManifoldEvaluationFactor() {}

  /**
   * @brief Construct a new ManifoldEvaluationFactor object.
   *
   * @param key Key for the state matrix parameterizing the function to estimate
   * via the BASIS.
   * @param z The measurement value.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param x The point at which the estimated function is evaluated.
   */
  ManifoldEvaluationFactor(Key key, const T &z, const SharedNoiseModel &model,
                           const size_t N, double x)
      : Base(key, z, model,
             typename BASIS::template ManifoldEvaluationFunctor<T>(N, x)) {}

  /**
   * @brief Construct a new ManifoldEvaluationFactor object.
   *
   * @param key Key for the state matrix parameterizing the function to estimate
   * via the BASIS.
   * @param z The measurement value.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param x The point at which the estimated function is evaluated.
   * @param a Lower bound for the polynomial.
   * @param b Upper bound for the polynomial.
   */
  ManifoldEvaluationFactor(Key key, const T &z, const SharedNoiseModel &model,
                           const size_t N, double x, double a, double b)
      : Base(
            key, z, model,
            typename BASIS::template ManifoldEvaluationFunctor<T>(N, x, a, b)) {
  }

  virtual ~ManifoldEvaluationFactor() {}
};

/**
 * A unary factor which enforces the evaluation of the derivative of a BASIS
 * polynomial at a specified point`x` is equal to the scalar measurement `z`.
 *
 * @param BASIS: The basis class to use e.g. Chebyshev2
 */
template <class BASIS>
class DerivativeFactor
    : public FunctorizedFactor<double, typename BASIS::Parameters> {
 private:
  using Base = FunctorizedFactor<double, typename BASIS::Parameters>;

 public:
  DerivativeFactor() {}

  /**
   * @brief Construct a new DerivativeFactor object.
   *
   * @param key The key to the ParameterMatrix which represents the basis
   * polynomial.
   * @param z The measurement value.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the basis polynomial.
   */
  DerivativeFactor(Key key, const double &z, const SharedNoiseModel &model,
                   const size_t N, double x)
      : Base(key, z, model, typename BASIS::DerivativeFunctor(N, x)) {}

  /**
   * @brief Construct a new DerivativeFactor object.
   *
   * @param key The key to the ParameterMatrix which represents the basis
   * polynomial.
   * @param z The measurement value.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the basis polynomial.
   * @param a Lower bound for the polynomial.
   * @param b Upper bound for the polynomial.
   */
  DerivativeFactor(Key key, const double &z, const SharedNoiseModel &model,
                   const size_t N, double x, double a, double b)
      : Base(key, z, model, typename BASIS::DerivativeFunctor(N, x, a, b)) {}

  virtual ~DerivativeFactor() {}
};

/**
 * A unary factor which enforces the evaluation of the derivative of a BASIS
 * polynomial at a specified point `x` is equal to the vector value `z`.
 *
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param M: Size of the evaluated state vector derivative.
 */
//TODO(Varun) remove template M
template <class BASIS, int M>
class VectorDerivativeFactor
    : public FunctorizedFactor<Vector, ParameterMatrix> {
 private:
  using Base = FunctorizedFactor<Vector, ParameterMatrix>;
  using Func = typename BASIS::template VectorDerivativeFunctor<M>;

 public:
  VectorDerivativeFactor() {}

  /**
   * @brief Construct a new VectorDerivativeFactor object.
   *
   * @param key The key to the ParameterMatrix which represents the basis
   * polynomial.
   * @param z The measurement value.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the basis polynomial.
   */
  VectorDerivativeFactor(Key key, const Vector &z,
                         const SharedNoiseModel &model, const size_t N,
                         double x)
      : Base(key, z, model, Func(N, x)) {}

  /**
   * @brief Construct a new VectorDerivativeFactor object.
   *
   * @param key The key to the ParameterMatrix which represents the basis
   * polynomial.
   * @param z The measurement value.
   * @param model The noise model.
   * @param N The degree of the polynomial.
   * @param x The point at which to evaluate the basis polynomial.
   * @param a Lower bound for the polynomial.
   * @param b Upper bound for the polynomial.
   */
  VectorDerivativeFactor(Key key, const Vector &z,
                         const SharedNoiseModel &model, const size_t N,
                         double x, double a, double b)
      : Base(key, z, model, Func(N, x, a, b)) {}

  virtual ~VectorDerivativeFactor() {}
};

/**
 * A unary factor which enforces the evaluation of the derivative of a BASIS
 * polynomial is equal to the scalar value at a specific index `i` of a
 * vector-valued measurement `z`.
 *
 * @param BASIS: The basis class to use e.g. Chebyshev2
 * @param P: Size of the control component derivative.
 */
// TODO(Varun) remove template P
template <class BASIS, int P>
class ComponentDerivativeFactor
    : public FunctorizedFactor<double, ParameterMatrix> {
 private:
  using Base = FunctorizedFactor<double, ParameterMatrix>;
  using Func = typename BASIS::template ComponentDerivativeFunctor<P>;

 public:
  ComponentDerivativeFactor() {}

  /**
   * @brief Construct a new ComponentDerivativeFactor object.
   *
   * @param key The key to the ParameterMatrix which represents the basis
   * polynomial.
   * @param z The scalar measurement value at a specific index `i` of the full
   * measurement vector.
   * @param model The degree of the polynomial.
   * @param N The degree of the polynomial.
   * @param i The index for the evaluated vector to give us the desired scalar
   * value.
   * @param x The point at which to evaluate the basis polynomial.
   */
  ComponentDerivativeFactor(Key key, const double &z,
                            const SharedNoiseModel &model, const size_t N,
                            size_t i, double x)
      : Base(key, z, model, Func(N, i, x)) {}

  /**
   * @brief Construct a new ComponentDerivativeFactor object.
   *
   * @param key The key to the ParameterMatrix which represents the basis
   * polynomial.
   * @param z The scalar measurement value at a specific index `i` of the full
   * measurement vector.
   * @param model The degree of the polynomial.
   * @param N The degree of the polynomial.
   * @param i The index for the evaluated vector to give us the desired scalar
   * value.
   * @param x The point at which to evaluate the basis polynomial.
   * @param a Lower bound for the polynomial.
   * @param b Upper bound for the polynomial.
   */
  ComponentDerivativeFactor(Key key, const double &z,
                            const SharedNoiseModel &model, const size_t N,
                            size_t i, double x, double a, double b)
      : Base(key, z, model, Func(N, i, x, a, b)) {}

  virtual ~ComponentDerivativeFactor() {}
};

}  // namespace gtsam
