/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FunctorizedFactor.h
 * @date May 31, 2020
 * @author Varun Agrawal
 **/

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>

namespace gtsam {

/**
 * Factor which evaluates provided unary functor and uses the result to compute
 * error with respect to the provided measurement.
 *
 * Template parameters are
 * @param R: The return type of the functor after evaluation.
 * @param T: The argument type for the functor.
 *
 * Example:
 *   Key key = Symbol('X', 0);
 *   auto model = noiseModel::Isotropic::Sigma(9, 1);
 *
 *   /// Functor that takes a matrix and multiplies every element by m
 *   class MultiplyFunctor {
 *     double m_; ///< simple multiplier
 *    public:
 *     MultiplyFunctor(double m) : m_(m) {}
 *     Matrix operator()(const Matrix &X,
 *              OptionalJacobian<-1, -1> H = boost::none) const {
 *       if (H)
 *         *H = m_ * Matrix::Identity(X.rows()*X.cols(), X.rows()*X.cols());
 *       return m_ * X;
 *     }
 *   };
 *
 *   Matrix measurement = Matrix::Identity(3, 3);
 *   double multiplier = 2.0;
 *
 *   FunctorizedFactor<Matrix, Matrix> factor(keyX, measurement, model,
 *     MultiplyFunctor(multiplier));
 */
template <typename R, typename T>
class FunctorizedFactor : public NoiseModelFactorN<T> {
 private:
  using Base = NoiseModelFactorN<T>;

  R measured_;  ///< value that is compared with functor return value
  SharedNoiseModel noiseModel_;                          ///< noise model
  std::function<R(T, OptionalMatrixType)> func_;  ///< functor instance

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;
  /** default constructor - only use for serialization */
  FunctorizedFactor() {}

  /** Construct with given x and the parameters of the basis
   *
   * @param key: Factor key
   * @param z: Measurement object of same type as that returned by functor
   * @param model: Noise model
   * @param func: The instance of the functor object
   */
  FunctorizedFactor(Key key, const R &z, const SharedNoiseModel &model,
                    const std::function<R(T, OptionalMatrixType)> func)
      : Base(model, key), measured_(z), noiseModel_(model), func_(func) {}

  ~FunctorizedFactor() override {}

  /// @return a deep copy of this factor
  NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new FunctorizedFactor<R, T>(*this)));
  }

  Vector evaluateError(const T &params, OptionalMatrixType H) const override {
    R x = func_(params, H);
    Vector error = traits<R>::Local(measured_, x);
    return error;
  }

  /// @name Testable
  /// @{
  void print(
      const std::string &s = "",
      const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << s << (s != "" ? " " : "") << "FunctorizedFactor("
              << keyFormatter(this->key1()) << ")" << std::endl;
    traits<R>::Print(measured_, "  measurement: ");
    std::cout << "  noise model sigmas: " << noiseModel_->sigmas().transpose()
              << std::endl;
  }

  bool equals(const NonlinearFactor &other, double tol = 1e-9) const override {
    const FunctorizedFactor<R, T> *e =
        dynamic_cast<const FunctorizedFactor<R, T> *>(&other);
    return e != nullptr && Base::equals(other, tol) &&
           traits<R>::Equals(this->measured_, e->measured_, tol);
  }
  /// @}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(measured_);
    ar &BOOST_SERIALIZATION_NVP(func_);
  }
};

/// traits
template <typename R, typename T>
struct traits<FunctorizedFactor<R, T>>
    : public Testable<FunctorizedFactor<R, T>> {};

/**
 * Helper function to create a functorized factor.
 *
 * Uses function template deduction to identify return type and functor type, so
 * template list only needs the functor argument type.
 */
template <typename T, typename R, typename FUNC>
FunctorizedFactor<R, T> MakeFunctorizedFactor(Key key, const R &z,
                                              const SharedNoiseModel &model,
                                              const FUNC func) {
  return FunctorizedFactor<R, T>(key, z, model, func);
}

/**
 * Factor which evaluates provided binary functor and uses the result to compute
 * error with respect to the provided measurement.
 *
 * Template parameters are
 * @param R: The return type of the functor after evaluation.
 * @param T1: The first argument type for the functor.
 * @param T2: The second argument type for the functor.
 */
template <typename R, typename T1, typename T2>
class FunctorizedFactor2 : public NoiseModelFactorN<T1, T2> {
 private:
  using Base = NoiseModelFactorN<T1, T2>;

  R measured_;  ///< value that is compared with functor return value
  SharedNoiseModel noiseModel_;  ///< noise model
  using FunctionType = std::function<R(T1, T2, OptionalMatrixType, OptionalMatrixType)>;
  FunctionType func_;  ///< functor instance

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;
  /** default constructor - only use for serialization */
  FunctorizedFactor2() {}

  /** Construct with given x and the parameters of the basis
   *
   * @param key: Factor key
   * @param z: Measurement object of same type as that returned by functor
   * @param model: Noise model
   * @param func: The instance of the functor object
   */
  FunctorizedFactor2(Key key1, Key key2, const R &z,
                     const SharedNoiseModel &model, const FunctionType func)
      : Base(model, key1, key2),
        measured_(z),
        noiseModel_(model),
        func_(func) {}

  ~FunctorizedFactor2() override {}

  /// @return a deep copy of this factor
  NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new FunctorizedFactor2<R, T1, T2>(*this)));
  }

  Vector evaluateError(
      const T1 &params1, const T2 &params2,
      OptionalMatrixType H1, OptionalMatrixType H2) const override {
    R x = func_(params1, params2, H1, H2);
    Vector error = traits<R>::Local(measured_, x);
    return error;
  }

  /// @name Testable
  /// @{
  void print(
      const std::string &s = "",
      const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << s << (s != "" ? " " : "") << "FunctorizedFactor2("
              << keyFormatter(this->key1()) << ", "
              << keyFormatter(this->key2()) << ")" << std::endl;
    traits<R>::Print(measured_, "  measurement: ");
    std::cout << "  noise model sigmas: " << noiseModel_->sigmas().transpose()
              << std::endl;
  }

  bool equals(const NonlinearFactor &other, double tol = 1e-9) const override {
    const FunctorizedFactor2<R, T1, T2> *e =
        dynamic_cast<const FunctorizedFactor2<R, T1, T2> *>(&other);
    return e && Base::equals(other, tol) &&
           traits<R>::Equals(this->measured_, e->measured_, tol);
  }
  /// @}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(measured_);
    ar &BOOST_SERIALIZATION_NVP(func_);
  }
};

/// traits
template <typename R, typename T1, typename T2>
struct traits<FunctorizedFactor2<R, T1, T2>>
    : public Testable<FunctorizedFactor2<R, T1, T2>> {};

/**
 * Helper function to create a functorized factor.
 *
 * Uses function template deduction to identify return type and functor type, so
 * template list only needs the functor argument type.
 */
template <typename T1, typename T2, typename R, typename FUNC>
FunctorizedFactor2<R, T1, T2> MakeFunctorizedFactor2(
    Key key1, Key key2, const R &z, const SharedNoiseModel &model,
    const FUNC func) {
  return FunctorizedFactor2<R, T1, T2>(key1, key2, z, model, func);
}

}  // namespace gtsam
