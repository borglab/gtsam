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
 * @author Varun Agrawal, Jing Dong, Frank Dellaert
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/basis/ParameterMatrix.h>

#include <iostream>

/**
 * This file supports creating continuous functions `f(x;p)` as a linear
 * combination of `basis functions` such as the Fourier basis on SO(2) or a set
 * of Chebyshev polynomials on [-1,1].
 *
 * In the expression `f(x;p)` the variable `x` is
 * the continuous argument at which the function is evaluated, and `p` are
 * the parameters which are coefficients of the different basis functions,
 * e.g. p = [4; 3; 2] => 4 + 3x + 2x^2 for a polynomial.
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
  - `CalculateWeights(size_t N, double x, double a=default, double b=default)`
  - `DerivativeWeights(size_t N, double x, double a=default, double b=default)`

  where `Weights` is an N*1 row vector which defines the basis values for the
 polynomial at the specified point `x`.

 E.g. A Fourier series would give the following:
  - `CalculateWeights` -> For N=5, the values for the bases:
        [1, cos(x), sin(x), cos(2x), sin(2x)]
  - `DerivativeWeights` -> For N=5, these are:
        [0, -sin(x), cos(x), -2sin(2x), 2cos(x)]

 Note that for a pseudo-spectral basis (as in Chebyshev2), the weights are
 instead the values for the Barycentric interpolation formula, since the values
 at the polynomial points (e.g. Chebyshev points) define the bases.
 */

namespace gtsam {

using Weights = Eigen::Matrix<double, 1, -1>; /* 1xN vector */

/**
 * @brief Function for computing the kronecker product of the 1*N Weight vector
 * `w` with the MxM identity matrix `I` efficiently.
 * The main reason for this is so we don't need to use Eigen's Unsupported
 * library.
 *
 * @tparam M Size of the identity matrix.
 * @param w The weights of the polynomial.
 * @return Mx(M*N) kronecker product [w(0)*I, w(1)*I, ..., w(N-1)*I]
 *
 * @ingroup basis
 */
Matrix kroneckerProductIdentity(size_t M, const Weights& w);

/**
 * CRTP Base class for function bases
 *  @ingroup basis
 */
template <typename DERIVED>
class Basis {
 public:
  /**
   * Calculate weights for all x in vector X.
   * Returns M*N matrix where M is the size of the vector X,
   * and N is the number of basis functions.
   */
  static Matrix WeightMatrix(size_t N, const Vector& X) {
    Matrix W(X.size(), N);
    for (int i = 0; i < X.size(); i++)
      W.row(i) = DERIVED::CalculateWeights(N, X(i));
    return W;
  }

  /**
   * @brief Calculate weights for all x in vector X, with interval [a,b].
   *
   * @param N The number of basis functions.
   * @param X The vector for which to compute the weights.
   * @param a The lower bound for the interval range.
   * @param b The upper bound for the interval range.
   * @return Returns M*N matrix where M is the size of the vector X.
   */
  static Matrix WeightMatrix(size_t N, const Vector& X, double a, double b) {
    Matrix W(X.size(), N);
    for (int i = 0; i < X.size(); i++)
      W.row(i) = DERIVED::CalculateWeights(N, X(i), a, b);
    return W;
  }

  /**
   * An instance of an EvaluationFunctor calculates f(x;p) at a given `x`,
   * applied to Parameters `p`.
   * This functor is used to evaluate a parameterized function at a given scalar
   * value x. When given a specific N*1 vector of Parameters, returns the scalar
   * value of the function at x, possibly with Jacobians wrpt the parameters.
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
    double apply(const typename DERIVED::Parameters& p,
                 OptionalJacobian<-1, -1> H = {}) const {
      if (H) *H = weights_;
      return (weights_ * p)(0);
    }

    /// c++ sugar
    double operator()(const typename DERIVED::Parameters& p,
                      OptionalJacobian<-1, -1> H = {}) const {
      return apply(p, H);  // might call apply in derived
    }

    void print(const std::string& s = "") const {
      std::cout << s << (s != "" ? " " : "") << weights_ << std::endl;
    }
  };

  /**
   * VectorEvaluationFunctor at a given x, applied to ParameterMatrix.
   * This functor is used to evaluate a parameterized function at a given scalar
   * value x. When given a specific M*N parameters, returns an M-vector the M
   * corresponding functions at x, possibly with Jacobians wrpt the parameters.
   */
  class VectorEvaluationFunctor : protected EvaluationFunctor {
   protected:
    using Jacobian = Eigen::Matrix<double, /*MxMN*/ -1, -1>;
    Jacobian H_;

    size_t M_;

    /**
     * Calculate the `M*(M*N)` Jacobian of this functor with respect to
     * the M*N parameter matrix `P`.
     * We flatten assuming column-major order, e.g., if N=3 and M=2, we have
     *      H =[ w(0) 0 w(1)  0 w(2)  0
     *           0  w(0)  0 w(1)  0 w(2) ]
     * i.e., the Kronecker product of weights_ with the MxM identity matrix.
     */
    void calculateJacobian() {
      H_ = kroneckerProductIdentity(M_, this->weights_);
    }

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// For serialization
    VectorEvaluationFunctor() {}

    /// Default Constructor
    VectorEvaluationFunctor(size_t M, size_t N, double x)
        : EvaluationFunctor(N, x), M_(M) {
      calculateJacobian();
    }

    /// Constructor, with interval [a,b]
    VectorEvaluationFunctor(size_t M, size_t N, double x, double a, double b)
        : EvaluationFunctor(N, x, a, b), M_(M) {
      calculateJacobian();
    }

    /// M-dimensional evaluation
    Vector apply(const ParameterMatrix& P,
                 OptionalJacobian</*MxN*/ -1, -1> H = {}) const {
      if (H) *H = H_;
      return P.matrix() * this->weights_.transpose();
    }

    /// c++ sugar
    Vector operator()(const ParameterMatrix& P,
                      OptionalJacobian</*MxN*/ -1, -1> H = {}) const {
      return apply(P, H);
    }
  };

  /**
   * Given a M*N Matrix of M-vectors at N polynomial points, an instance of
   * VectorComponentFunctor computes the N-vector value for a specific row
   * component of the M-vectors at all the polynomial points.
   *
   * This component is specified by the row index i, with 0<i<M.
   */
  class VectorComponentFunctor : public EvaluationFunctor {
   protected:
    using Jacobian = Eigen::Matrix<double, /*1xMN*/ 1, -1>;
    Jacobian H_;

    size_t M_;
    size_t rowIndex_;

    /*
     * Calculate the `1*(M*N)` Jacobian of this functor with respect to
     * the M*N parameter matrix `P`.
     * We flatten assuming column-major order, e.g., if N=3 and M=2, we have
     *      H=[w(0) 0    w(1) 0    w(2) 0]    for rowIndex==0
     *      H=[0    w(0) 0    w(1) 0    w(2)] for rowIndex==1
     * i.e., one row of the Kronecker product of weights_ with the
     * MxM identity matrix. See also VectorEvaluationFunctor.
     */
    void calculateJacobian(size_t N) {
      H_.setZero(1, M_ * N);
      for (int j = 0; j < EvaluationFunctor::weights_.size(); j++)
        H_(0, rowIndex_ + j * M_) = EvaluationFunctor::weights_(j);
    }

   public:
    /// For serialization
    VectorComponentFunctor() {}

    /// Construct with row index
    VectorComponentFunctor(size_t M, size_t N, size_t i, double x)
        : EvaluationFunctor(N, x), M_(M), rowIndex_(i) {
      calculateJacobian(N);
    }

    /// Construct with row index and interval
    VectorComponentFunctor(size_t M, size_t N, size_t i, double x, double a,
                           double b)
        : EvaluationFunctor(N, x, a, b), M_(M), rowIndex_(i) {
      calculateJacobian(N);
    }

    /// Calculate component of component rowIndex_ of P
    double apply(const ParameterMatrix& P,
                 OptionalJacobian</*1xMN*/ -1, -1> H = {}) const {
      if (H) *H = H_;
      return P.row(rowIndex_) * EvaluationFunctor::weights_.transpose();
    }

    /// c++ sugar
    double operator()(const ParameterMatrix& P,
                      OptionalJacobian</*1xMN*/ -1, -1> H = {}) const {
      return apply(P, H);
    }
  };

  /**
   * Manifold EvaluationFunctor at a given x, applied to ParameterMatrix.
   * This functor is used to evaluate a parameterized function at a given scalar
   * value x. When given a specific M*N parameters, returns an M-vector the M
   * corresponding functions at x, possibly with Jacobians wrpt the parameters.
   *
   * The difference with the VectorEvaluationFunctor is that after computing the
   * M*1 vector xi=F(x;P), with x a scalar and P the M*N parameter vector, we
   * also retract xi back to the T manifold.
   * For example, if T==Rot3, then we first compute a 3-vector xi using x and P,
   * and then map that 3-vector xi back to the Rot3 manifold, yielding a valid
   * 3D rotation.
   */
  template <class T>
  class ManifoldEvaluationFunctor : public VectorEvaluationFunctor {
    enum { M = traits<T>::dimension };
    using Base = VectorEvaluationFunctor;

   public:
    /// For serialization
    ManifoldEvaluationFunctor() {}

    /// Default Constructor
    ManifoldEvaluationFunctor(size_t N, double x) : Base(M, N, x) {}

    /// Constructor, with interval [a,b]
    ManifoldEvaluationFunctor(size_t N, double x, double a, double b)
        : Base(M, N, x, a, b) {}

    /// Manifold evaluation
    T apply(const ParameterMatrix& P,
            OptionalJacobian</*MxMN*/ -1, -1> H = {}) const {
      // Interpolate the M-dimensional vector to yield a vector in tangent space
      Eigen::Matrix<double, M, 1> xi = Base::operator()(P, H);

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
    T operator()(const ParameterMatrix& P,
                 OptionalJacobian</*MxN*/ -1, -1> H = {}) const {
      return apply(P, H);  // might call apply in derived
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

  /**
   * An instance of a DerivativeFunctor calculates f'(x;p) at a given `x`,
   * applied to Parameters `p`.
   * When given a scalar value x and a specific N*1 vector of Parameters,
   * this functor returns the scalar derivative value of the function at x,
   * possibly with Jacobians wrpt the parameters.
   */
  class DerivativeFunctor : protected DerivativeFunctorBase {
   public:
    /// For serialization
    DerivativeFunctor() {}

    DerivativeFunctor(size_t N, double x) : DerivativeFunctorBase(N, x) {}

    DerivativeFunctor(size_t N, double x, double a, double b)
        : DerivativeFunctorBase(N, x, a, b) {}

    double apply(const typename DERIVED::Parameters& p,
                 OptionalJacobian</*1xN*/ -1, -1> H = {}) const {
      if (H) *H = this->weights_;
      return (this->weights_ * p)(0);
    }
    /// c++ sugar
    double operator()(const typename DERIVED::Parameters& p,
                      OptionalJacobian</*1xN*/ -1, -1> H = {}) const {
      return apply(p, H);  // might call apply in derived
    }
  };

  /**
   * VectorDerivativeFunctor at a given x, applied to ParameterMatrix.
   *
   * This functor is used to evaluate the derivatives of a parameterized
   * function at a given scalar value x. When given a specific M*N parameters,
   * returns an M-vector the M corresponding function derivatives at x, possibly
   * with Jacobians wrpt the parameters.
   */
  class VectorDerivativeFunctor : protected DerivativeFunctorBase {
   protected:
    using Jacobian = Eigen::Matrix<double, /*MxMN*/ -1, -1>;
    Jacobian H_;

    size_t M_;

    /**
     * Calculate the `M*(M*N)` Jacobian of this functor with respect to
     * the M*N parameter matrix `P`.
     * We flatten assuming column-major order, e.g., if N=3 and M=2, we have
     *      H =[ w(0) 0 w(1)  0 w(2)  0
     *           0  w(0)  0 w(1)  0 w(2) ]
     * i.e., the Kronecker product of weights_ with the MxM identity matrix.
     */
    void calculateJacobian() {
      H_ = kroneckerProductIdentity(M_, this->weights_);
    }

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// For serialization
    VectorDerivativeFunctor() {}

    /// Default Constructor
    VectorDerivativeFunctor(size_t M, size_t N, double x)
        : DerivativeFunctorBase(N, x), M_(M) {
      calculateJacobian();
    }

    /// Constructor, with optional interval [a,b]
    VectorDerivativeFunctor(size_t M, size_t N, double x, double a, double b)
        : DerivativeFunctorBase(N, x, a, b), M_(M) {
      calculateJacobian();
    }

    Vector apply(const ParameterMatrix& P,
                 OptionalJacobian</*MxMN*/ -1, -1> H = {}) const {
      if (H) *H = H_;
      return P.matrix() * this->weights_.transpose();
    }
    /// c++ sugar
    Vector operator()(const ParameterMatrix& P,
                      OptionalJacobian</*MxMN*/ -1, -1> H = {}) const {
      return apply(P, H);
    }
  };

  /**
   * Given a M*N Matrix of M-vectors at N polynomial points, an instance of
   * ComponentDerivativeFunctor computes the N-vector derivative for a specific
   * row component of the M-vectors at all the polynomial points.
   *
   * This component is specified by the row index i, with 0<i<M.
   */
  class ComponentDerivativeFunctor : protected DerivativeFunctorBase {
   protected:
    using Jacobian = Eigen::Matrix<double, /*1xMN*/ 1, -1>;
    Jacobian H_;

    size_t M_;
    size_t rowIndex_;

    /*
     * Calculate the `1*(M*N)` Jacobian of this functor with respect to
     * the M*N parameter matrix `P`.
     * We flatten assuming column-major order, e.g., if N=3 and M=2, we have
     *      H=[w(0) 0    w(1) 0    w(2) 0]    for rowIndex==0
     *      H=[0    w(0) 0    w(1) 0    w(2)] for rowIndex==1
     * i.e., one row of the Kronecker product of weights_ with the
     * MxM identity matrix. See also VectorDerivativeFunctor.
     */
    void calculateJacobian(size_t N) {
      H_.setZero(1, M_ * N);
      for (int j = 0; j < this->weights_.size(); j++)
        H_(0, rowIndex_ + j * M_) = this->weights_(j);
    }

   public:
    /// For serialization
    ComponentDerivativeFunctor() {}

    /// Construct with row index
    ComponentDerivativeFunctor(size_t M, size_t N, size_t i, double x)
        : DerivativeFunctorBase(N, x), M_(M), rowIndex_(i) {
      calculateJacobian(N);
    }

    /// Construct with row index and interval
    ComponentDerivativeFunctor(size_t M, size_t N, size_t i, double x, double a,
                               double b)
        : DerivativeFunctorBase(N, x, a, b), M_(M), rowIndex_(i) {
      calculateJacobian(N);
    }
    /// Calculate derivative of component rowIndex_ of F
    double apply(const ParameterMatrix& P,
                 OptionalJacobian</*1xMN*/ -1, -1> H = {}) const {
      if (H) *H = H_;
      return P.row(rowIndex_) * this->weights_.transpose();
    }
    /// c++ sugar
    double operator()(const ParameterMatrix& P,
                      OptionalJacobian</*1xMN*/ -1, -1> H = {}) const {
      return apply(P, H);
    }
  };
};

}  // namespace gtsam
