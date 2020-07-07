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
class GTSAM_EXPORT FunctorizedFactor : public NoiseModelFactor1<T> {
 private:
  using Base = NoiseModelFactor1<T>;

  R measured_;  ///< value that is compared with functor return value
  SharedNoiseModel noiseModel_;                          ///< noise model
  std::function<R(T, boost::optional<Matrix &>)> func_;  ///< functor instance

 public:
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
                    const std::function<R(T, boost::optional<Matrix &>)> func)
      : Base(model, key), measured_(z), noiseModel_(model), func_(func) {}

  virtual ~FunctorizedFactor() {}

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new FunctorizedFactor<R, T>(*this)));
  }

  Vector evaluateError(const T &params,
                       boost::optional<Matrix &> H = boost::none) const {
    R x = func_(params, H);
    Vector error = traits<R>::Local(measured_, x);
    return error;
  }

  /// @name Testable
  /// @{
  void print(const std::string &s = "",
             const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
    Base::print(s, keyFormatter);
    std::cout << s << (s != "" ? " " : "") << "FunctorizedFactor("
              << keyFormatter(this->key()) << ")" << std::endl;
    traits<R>::Print(measured_, "  measurement: ");
    std::cout << "  noise model sigmas: " << noiseModel_->sigmas().transpose()
              << std::endl;
  }

  virtual bool equals(const NonlinearFactor &other, double tol = 1e-9) const {
    const FunctorizedFactor<R, T> *e =
        dynamic_cast<const FunctorizedFactor<R, T> *>(&other);
    const bool base = Base::equals(*e, tol);
    return e && Base::equals(other, tol) &&
           traits<R>::Equals(this->measured_, e->measured_, tol);
  }
  /// @}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
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

}  // namespace gtsam
