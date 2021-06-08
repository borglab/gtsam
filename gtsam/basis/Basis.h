/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Basis.h
 * @brief Compute an interpolating basis
 * @author Varun Agrawal, Frank Dellaert
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/basis/ParameterMatrix.h>

#include <iostream>
#include <unsupported/Eigen/KroneckerProduct>

/**
 * This file supports creating continuous functions `f(x;p)` as a linear
 * combination of `basis functions` such as the Fourier basis on SO(2) or a set
 * of Chebyshev polynomials on [-1,1].
 * In the expression `f(x;p)` the variable `x` is
 * the continuous argument at which teh function is evaluated, and `p` are
 * parameters, e.g., coefficients used to weight the different basis functions.
 * However, different parameterizations are also possible.

 * The `Basis` class below defines a number of functors that can be used to
 * evaluate `f(x;p)` at a given `x`, and these functors also calculate
 * the Jacobian of `f(x;p)` with respect to the parameters `p`.
 * This is actually the most important calculation, as it will allow GTSAM
 * to optimize over the parameters `p`.

 * This functionality is implemented using the `CRTP` or "Curiously recurring
 * template pattern" C++ idiom, which is a meta-programming technique in which
 * the derived class is passed as a template argument to `Basis<DERIVED>`.
 * The DERIVED class is assumed to satisfy a C++ concept,
 * i.e., we expect it to define the following types and methods:

  - type `Parameters`: the parameters `p` in f(x;p)
  - type `Weights`: vector of partial derivatives of `f(x;p)` wrt `p`
  - `CalculateWeights(size_t N, double x, double a=default, double b=default)`
  - `DerivativeWeights(size_t N, double x, double a=default, double b=default)`
 */

namespace gtsam {

using Weights = Eigen::Matrix<double, 1, -1>; /* 1xN vector */

/// CRTP Base class for function bases
template <typename DERIVED>
class Basis {
 public:
  /**
   * Call weights for all x in vector X.
   * Returns M*N matrix where M is the size of the vector X,
   * and N is the number of basis function.
   */
  static Matrix WeightMatrix(size_t N, const Vector& X) {
    Matrix W(X.size(), N);
    for (int i = 0; i < X.size(); i++)
      W.row(i) = DERIVED::CalculateWeights(N, X(i));
    return W;
  }

  /**
   * Call weights for all x in vector X, with interval [a,b].
   * Returns M*N matrix where M is the size of the vector X,
   * and N is the number of basis function.
   */
  static Matrix WeightMatrix(size_t N, const Vector& X, double a, double b) {
    Matrix W(X.size(), N);
    for (int i = 0; i < X.size(); i++)
      W.row(i) = DERIVED::CalculateWeights(N, X(i), a, b);
    return W;
  }

  /**
   * EvaluationFunctor calculate f(x;p) at a given `x`, applied to Parameters
   * `p`. This functor is used to evaluate a parameterized function at a given
   * scalar value x. When given a specific N*1 vector of Parameters, returns the
   * scalar value of the function at x, possibly with Jacobian wrpt the
   * parameters.
   */
  class EvaluationFunctor {
   protected:
    Weights weights_;

   public:
    /// For serialization
    EvaluationFunctor() {}

    /// Constructor with interval [a,b]
    EvaluationFunctor(size_t N, double x)
        : weights_(DERIVED::CalculateWeights(N, x)) {}

    /// Constructor with interval [a,b]
    EvaluationFunctor(size_t N, double x, double a, double b)
        : weights_(DERIVED::CalculateWeights(N, x, a, b)) {}

    /// Regular 1D evaluation
    double apply(const typename DERIVED::Parameters& f,
                 OptionalJacobian<-1, -1> H = boost::none) const {
      if (H) *H = weights_;
      return (weights_ * f)(0);
    }

    /// c++ sugar
    double operator()(const typename DERIVED::Parameters& f,
                      OptionalJacobian<-1, -1> H = boost::none) const {
      return apply(f, H);  // might call apply in derived
    }

    void print(const std::string& s = "") const {
      std::cout << s << (s != "" ? " " : "") << weights_ << std::endl;
    }
  };

  /**
   * VectorEvaluationFunctor at a given x, applied to ParameterMatrix<M>.
   * This functor is used to evaluate a parameterized function at a given scalar
   * value x. When given a specific M*N parameters, returns an M-vector the M
   * corresponding functions at x, possibly with Jacobian wrpt the parameters.
   */
  template <int M>
  class VectorEvaluationFunctor : protected EvaluationFunctor {
   protected:
    using VectorM = Eigen::Matrix<double, M, 1>;
    using Jacobian = Eigen::Matrix<double, /*MxMN*/ M, -1>;
    Jacobian H_;

    void calculateJacobian() {
      using MatrixM = Eigen::Matrix<double, M, M>;
      H_ = Eigen::kroneckerProduct(this->weights_, MatrixM::Identity());
    }

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// For serialization
    VectorEvaluationFunctor() {}

    /// Default Constructor
    VectorEvaluationFunctor(size_t N, double x) : EvaluationFunctor(N, x) {
      calculateJacobian();
    }

    /// Constructor, with interval [a,b]
    VectorEvaluationFunctor(size_t N, double x, double a, double b)
        : EvaluationFunctor(N, x, a, b) {
      calculateJacobian();
    }

    /// M-dimensional evaluation
    VectorM apply(const ParameterMatrix<M>& F,
                  OptionalJacobian</*MxN*/ -1, -1> H = boost::none) const {
      if (H) *H = H_;
      return F.matrix() * this->weights_.transpose();
    }

    /// c++ sugar
    VectorM operator()(const ParameterMatrix<M>& F,
                       OptionalJacobian</*MxN*/ -1, -1> H = boost::none) const {
      return apply(F, H);
    }
  };

  /**
   * Given M*N Matrix of M-vectors at N Chebyshev points, predict component of
   * row vector at given i, with 0<i<M.
   */
  template <int M>
  class VectorComponentFunctor : public EvaluationFunctor {
   protected:
    using Jacobian = Eigen::Matrix<double, /*1xMN*/ 1, -1>;
    size_t rowIndex_;
    Jacobian H_;
    void calculateJacobian(size_t N) {
      H_.setZero(1, M * N);
      for (int j = 0; j < EvaluationFunctor::weights_.size(); j++)
        H_(0, rowIndex_ + j * M) = EvaluationFunctor::weights_(j);
    }

   public:
    /// For serialization
    VectorComponentFunctor() {}

    /// Construct with row index
    VectorComponentFunctor(size_t N, size_t i, double x)
        : EvaluationFunctor(N, x), rowIndex_(i) {
      calculateJacobian(N);
    }

    /// Construct with row index and interval
    VectorComponentFunctor(size_t N, size_t i, double x, double a, double b)
        : EvaluationFunctor(N, x, a, b), rowIndex_(i) {
      calculateJacobian(N);
    }

    /// Calculate component of component rowIndex_ of F
    double apply(const ParameterMatrix<M>& F,
                 OptionalJacobian</*1xMN*/ -1, -1> H = boost::none) const {
      if (H) *H = H_;
      return F.row(rowIndex_) * EvaluationFunctor::weights_.transpose();
    }

    /// c++ sugar
    double operator()(const ParameterMatrix<M>& F,
                      OptionalJacobian</*1xMN*/ -1, -1> H = boost::none) const {
      return apply(F, H);
    }
  };

  /**
   * Manifold EvaluationFunctor at a given x, applied to ParameterMatrix<M>.
   * This functor is used to evaluate a parameterized function at a given scalar
   * value x. When given a specific M*N parameters, returns an M-vector the M
   * corresponding functions at x, possibly with Jacobian wrpt the parameters.
   */
  /// Manifold interpolation
  template <class T>
  class ManifoldEvaluationFunctor
      : public VectorEvaluationFunctor<traits<T>::dimension> {
    enum { M = traits<T>::dimension };
    using Base = VectorEvaluationFunctor<M>;

   public:
    /// For serialization
    ManifoldEvaluationFunctor() {}

    /// Default Constructor
    ManifoldEvaluationFunctor(size_t N, double x) : Base(N, x) {}

    /// Constructor, with interval [a,b]
    ManifoldEvaluationFunctor(size_t N, double x, double a, double b)
        : Base(N, x, a, b) {}

    /// Manifold evaluation
    T apply(const ParameterMatrix<M>& F,
            OptionalJacobian</*MxMN*/ -1, -1> H = boost::none) const {
      // Interpolate the M-dimensional vector to yield a vector in tangent space
      Eigen::Matrix<double, M, 1> xi = Base::operator()(F, H);

      // Now call retract with this M-vector, possibly with derivatives
      Eigen::Matrix<double, M, M> D_result_xi;
      T result = T::ChartAtOrigin::Retract(xi, H ? &D_result_xi : 0);

      // Finally, if derivatives are asked, apply chain rule where H is Mx(M*N)
      // derivative of interpolation and D_result_xi is MxM derivative of
      // retract.
      if (H) *H = D_result_xi * (*H);

      // and return a T
      return result;
    }

    /// c++ sugar
    T operator()(const ParameterMatrix<M>& F,
                 OptionalJacobian</*MxN*/ -1, -1> H = boost::none) const {
      return apply(F, H);  // might call apply in derived
    }
  };

  /// Base class for functors below that calculate derivative weights
  class DerivativeFunctorBase {
   protected:
    Weights weights_;

   public:
    /// For serialization
    DerivativeFunctorBase() {}

    DerivativeFunctorBase(size_t N, double x)
        : weights_(DERIVED::DerivativeWeights(N, x)) {}

    DerivativeFunctorBase(size_t N, double x, double a, double b)
        : weights_(DERIVED::DerivativeWeights(N, x, a, b)) {}

    void print(const std::string& s = "") const {
      std::cout << s << (s != "" ? " " : "") << weights_ << std::endl;
    }
  };

  /// Given values f at the Chebyshev points, predict derivative at x
  class DerivativeFunctor : protected DerivativeFunctorBase {
   public:
    /// For serialization
    DerivativeFunctor() {}

    DerivativeFunctor(size_t N, double x) : DerivativeFunctorBase(N, x) {}

    DerivativeFunctor(size_t N, double x, double a, double b)
        : DerivativeFunctorBase(N, x, a, b) {}

    double apply(const typename DERIVED::Parameters& f,
                 OptionalJacobian</*1xN*/ -1, -1> H = boost::none) const {
      if (H) *H = this->weights_;
      return (this->weights_ * f)(0);
    }
    /// c++ sugar
    double operator()(const typename DERIVED::Parameters& f,
                      OptionalJacobian</*1xN*/ -1, -1> H = boost::none) const {
      return apply(f, H);  // might call apply in derived
    }
  };

  /// Compute derivative vector of ParameterMatrix at specified point.
  template <int M>
  class VectorDerivativeFunctor : protected DerivativeFunctorBase {
   protected:
    using VectorM = Eigen::Matrix<double, M, 1>;
    using Jacobian = Eigen::Matrix<double, /*MxMN*/ M, -1>;
    Jacobian H_;

    void calculateJacobian() {
      using MatrixM = Eigen::Matrix<double, M, M>;
      H_ = Eigen::kroneckerProduct(this->weights_, MatrixM::Identity());
    }

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// For serialization
    VectorDerivativeFunctor() {}

    /// Default Constructor
    VectorDerivativeFunctor(size_t N, double x) : DerivativeFunctorBase(N, x) {
      calculateJacobian();
    }

    /// Constructor, with optional interval [a,b]
    VectorDerivativeFunctor(size_t N, double x, double a, double b)
        : DerivativeFunctorBase(N, x, a, b) {
      calculateJacobian();
    }

    VectorM apply(const ParameterMatrix<M>& F,
                  OptionalJacobian</*MxMN*/ -1, -1> H = boost::none) const {
      if (H) *H = H_;
      return F.matrix() * this->weights_.transpose();
    }
    /// c++ sugar
    VectorM operator()(
        const ParameterMatrix<M>& F,
        OptionalJacobian</*MxMN*/ -1, -1> H = boost::none) const {
      return apply(F, H);
    }
  };

  /**
   * Given M*N Matrix of M-vectors at N Chebyshev points, predict derivative for
   * given row i, with 0<=i<M.
   */
  template <int M>
  class ComponentDerivativeFunctor : protected DerivativeFunctorBase {
   protected:
    using Jacobian = Eigen::Matrix<double, /*1xMN*/ 1, -1>;
    size_t rowIndex_;
    Jacobian H_;

    void calculateJacobian(size_t N) {
      H_.setZero(1, M * N);
      for (int j = 0; j < this->weights_.size(); j++)
        H_(0, rowIndex_ + j * M) = this->weights_(j);
    }

   public:
    /// For serialization
    ComponentDerivativeFunctor() {}

    /// Construct with row index
    ComponentDerivativeFunctor(size_t N, size_t i, double x)
        : DerivativeFunctorBase(N, x), rowIndex_(i) {
      calculateJacobian(N);
    }

    /// Construct with row index and interval
    ComponentDerivativeFunctor(size_t N, size_t i, double x, double a, double b)
        : DerivativeFunctorBase(N, x, a, b), rowIndex_(i) {
      calculateJacobian(N);
    }
    /// Calculate derivative of component rowIndex_ of F
    double apply(const ParameterMatrix<M>& F,
                 OptionalJacobian</*1xMN*/ -1, -1> H = boost::none) const {
      if (H) *H = H_;
      return F.row(rowIndex_) * this->weights_.transpose();
    }
    /// c++ sugar
    double operator()(const ParameterMatrix<M>& F,
                      OptionalJacobian</*1xMN*/ -1, -1> H = boost::none) const {
      return apply(F, H);
    }
  };

  // Vector version for MATLAB :-(
  static double Derivative(double x, const Vector& f,  //
                           OptionalJacobian</*1xN*/ -1, -1> H = boost::none) {
    return DerivativeFunctor(x)(f.transpose(), H);
  }
};

}  // namespace gtsam
