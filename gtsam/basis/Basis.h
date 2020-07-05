/**
 * @file Basis.h
 * @brief Compute an interpolating basis
 * @author Varun Agrawal
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>

#include <gtsam/3rdparty/Eigen/unsupported/Eigen/KroneckerProduct>

/*
 *  Concept of a Basis:
 *    - type Weights, Parameters
 *    - CalculateWeights(size_t N, double a=default, double b=default)
 */

namespace gtsam {

/// CRTP Base class for function bases
template <typename DERIVED>
class Basis {
 public:
  /// Call weights for all x in vector, returns M*N matrix where M is the size
  /// of the vector X.
  static Matrix WeightMatrix(size_t N, const Vector& X); 

  /// Call weights for all x in vector, with interval [a,b]
  /// Returns M*N matrix where M is the size of the vector X.
  static Matrix WeightMatrix(size_t N, const Vector& X, double a, double b);

  /**
   * EvaluationFunctor at a given x, applied to Parameters.
   * This functor is used to evaluate a parameterized function at a given scalar
   * value x. When given a specific N*1 vector of Parameters, returns the scalar
   * value of the function at x, possibly with Jacobian wrpt the parameters.
   */
  class EvaluationFunctor {
   protected:
    typename DERIVED::Weights weights_;

   public:
    // Used by FunctorizedFactor
    using argument_type = Vector;
    using return_type = double;

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

    void print(const std::string& s = "") const;
  };

  /**
   * VectorEvaluationFunctor at a given x, applied to MatrixMN.
   * This functor is used to evaluate a parameterized function at a given scalar
   * value x. When given a specific M*N parameters, returns an M-vector the M
   * corresponding functions at x, possibly with Jacobian wrpt the parameters.
   */
  template <int M>
  class VectorEvaluationFunctor : protected EvaluationFunctor {
   protected:
    typedef Eigen::Matrix<double, M, 1> VectorM;
    typedef Eigen::Matrix<double, M, -1> MatrixMN;
    typedef Eigen::Matrix<double, /*MxMN*/ M, -1> Jacobian;
    Jacobian H_;
    void calculateJacobian() {
      typedef Eigen::Matrix<double, M, M> MatrixM;
      H_ = Eigen::kroneckerProduct(this->weights_, MatrixM::Identity());
    }

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Used by FunctorizedFactor
    using argument_type = Matrix;
    using return_type = Vector;

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
    VectorM apply(const MatrixMN& F,
                  OptionalJacobian</*M*N*/ -1, -1> H = boost::none) const {
      if (H) *H = H_;
      return F * this->weights_.transpose();
    }

    /// c++ sugar
    VectorM operator()(const MatrixMN& F,
                       OptionalJacobian</*M*N*/ -1, -1> H = boost::none) const {
      return apply(F, H);
    }
  };

  /**
   * Given M*N Matrix of M-vectors at N Chebyshev points, predict component for
   * given row i, with 0<i<M.
   */
  template <int M>
  class ComponentEvaluationFunctor : public EvaluationFunctor {
   protected:
    typedef Eigen::Matrix<double, M, -1> MatrixMN;
    typedef Eigen::Matrix<double, /*1xMN*/ 1, -1> Jacobian;
    size_t rowIndex_;
    Jacobian H_;
    void calculateJacobian(size_t N) {
      H_.setZero(1, M * N);
      for (int j = 0; j < EvaluationFunctor::weights_.size(); j++)
        H_(0, rowIndex_ + j * M) = EvaluationFunctor::weights_(j);
    }

   public:
    // Used by FunctorizedFactor
    using argument_type = Matrix;
    using return_type = double;

    /// Construct with row index
    ComponentEvaluationFunctor(size_t N, size_t i, double x)
        : EvaluationFunctor(N, x), rowIndex_(i) {
      calculateJacobian(N);
    }

    /// Construct with row index and interval
    ComponentEvaluationFunctor(size_t N, size_t i, double x, double a, double b)
        : EvaluationFunctor(N, x, a, b), rowIndex_(i) {
      calculateJacobian(N);
    }

    /// Calculate component of component rowIndex_ of F
    double apply(const MatrixMN& F,
                 OptionalJacobian</*1xMN*/ -1, -1> H = boost::none) const {
      if (H) *H = H_;
      return F.row(rowIndex_) * EvaluationFunctor::weights_.transpose();
    }

    /// c++ sugar
    double operator()(const MatrixMN& F,
                      OptionalJacobian</*1xMN*/ -1, -1> H = boost::none) const {
      return apply(F, H);
    }
  };

  /**
   * Manifold EvaluationFunctor at a given x, applied to MatrixMN.
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
    // Used by FunctorizedFactor
    using argument_type = Matrix;
    using return_type = T;

    /// Default Constructor
    ManifoldEvaluationFunctor(size_t N, double x) : Base(N, x) {}

    /// Constructor, with interval [a,b]
    ManifoldEvaluationFunctor(size_t N, double x, double a, double b)
        : Base(N, x, a, b) {}

    /// Manifold evaluation
    T apply(const typename Base::MatrixMN& F,
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
    T operator()(const typename Base::MatrixMN& F,
                 OptionalJacobian</*M*N*/ -1, -1> H = boost::none) const {
      return apply(F, H);  // might call apply in derived
    }
  };

  /// Base class for functors below that calculates weights
  class DerivativeFunctorBase {
   protected:
    typename DERIVED::Weights weights_;

   public:
    DerivativeFunctorBase(size_t N, double x)
        : weights_(DERIVED::DerivativeWeights(N, x)) {}

    DerivativeFunctorBase(size_t N, double x, double a, double b)
        : weights_(DERIVED::DerivativeWeights(N, x, a, b)) {}

    void print(const std::string& s = "") const;
  };

  /// Given values f at the Chebyshev points, predict derivative at x
  class DerivativeFunctor : protected DerivativeFunctorBase {
   public:
    // Used by FunctorizedFactor
    using argument_type = typename DERIVED::Parameters;
    using return_type = double;

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

  /// Vector interpolation, e.g. given 3*N matrix yields 3-vector
  template <int M>
  class VectorDerivativeFunctor : protected DerivativeFunctorBase {
   protected:
    typedef Eigen::Matrix<double, M, 1> VectorM;
    typedef Eigen::Matrix<double, M, -1> MatrixMN;
    typedef Eigen::Matrix<double, /*MxMN*/ M, -1> Jacobian;
    Jacobian H_;
    void calculateJacobian() {
      typedef Eigen::Matrix<double, M, M> MatrixM;
      H_ = Eigen::kroneckerProduct(this->weights_, MatrixM::Identity());
    }

   public:
    // Used by FunctorizedFactor
    using argument_type = Matrix;
    using return_type = Vector;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Default Constructor
    VectorDerivativeFunctor(size_t N, double x) : DerivativeFunctorBase(N, x) {
      calculateJacobian();
    }

    /// Constructor, with optional interval [a,b]
    VectorDerivativeFunctor(size_t N, double x, double a, double b)
        : DerivativeFunctorBase(N, x, a, b) {
      calculateJacobian();
    }

    VectorM apply(const MatrixMN& F,
                  OptionalJacobian</*MxMN*/ -1, -1> H = boost::none) const {
      if (H) *H = H_;
      return F * this->weights_.transpose();
    }
    /// c++ sugar
    VectorM operator()(const MatrixMN& F, OptionalJacobian</*MxMN*/ -1, -1> H =
                                              boost::none) const {
      return apply(F, H);
    }
  };

  /**
   * Given M*N Matrix of M-vectors at N Chebyshev points, predict derivative for
   * given row i, with 0<i<M.
   */
  template <int M>
  class ComponentDerivativeFunctor : protected DerivativeFunctorBase {
   protected:
    typedef Eigen::Matrix<double, M, -1> MatrixMN;
    typedef Eigen::Matrix<double, /*1xMN*/ 1, -1> Jacobian;
    size_t rowIndex_;
    Jacobian H_;
    void calculateJacobian(size_t N) {
      H_.setZero(1, M * N);
      for (int j = 0; j < this->weights_.size(); j++)
        H_(0, rowIndex_ + j * M) = this->weights_(j);
    }

   public:
    // Used by FunctorizedFactor
    using argument_type = Matrix;
    using return_type = double;

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
    double apply(const MatrixMN& F,
                 OptionalJacobian</*1xMN*/ -1, -1> H = boost::none) const {
      if (H) *H = H_;
      return F.row(rowIndex_) * this->weights_.transpose();
    }
    /// c++ sugar
    double operator()(const MatrixMN& F,
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
