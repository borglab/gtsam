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
 * Factor which evaluates functor and uses the result to compute
 * error on provided measurement.
 * The provided FUNCTOR should provide two type aliases: `argument_type` which
 * corresponds to the type of input it accepts and `return_type` which indicates
 * the type of the return value. This factor uses those type values to construct
 * the functor.
 *
 * Template parameters are
 * @param FUNCTOR: A class which operates as a functor.
 * 
 * Example:
 *   Key key = Symbol('X', 0);
 *
 *   auto model = noiseModel::Isotropic::Sigma(9, 1);
 *   /// Functor that takes a matrix and multiplies every element by m
 *   class MultiplyFunctor {
 *     double m_; ///< simple multiplier
 *    public:
 *     using argument_type = Matrix;
 *     using return_type = Matrix;
 *     MultiplyFunctor(double m) : m_(m) {}
 *     Matrix operator()(const Matrix &X,
 *              OptionalJacobian<-1, -1> H = boost::none) const {
 *       if (H) *H = m_ * Matrix::Identity(X.rows()*X.cols(), X.rows()*X.cols());
 *       return m_ * X;
 *     }
 *   };
 *
 *   Matrix measurement = Matrix::Identity(3, 3);
 *   double multiplier = 2.0;
 *   FunctorizedFactor<MultiplyFunctor> factor(keyX, measurement, model, multiplier);
 */
template <typename FUNCTOR>
class GTSAM_EXPORT FunctorizedFactor
    : public NoiseModelFactor1<typename FUNCTOR::argument_type> {
private:
  using T = typename FUNCTOR::argument_type;
  using Base = NoiseModelFactor1<T>;

  typename FUNCTOR::return_type
      measured_; ///< value that is compared with functor return value
  SharedNoiseModel noiseModel_; ///< noise model
  FUNCTOR func_;                ///< functor instance

public:
  /** default constructor - only use for serialization */
  FunctorizedFactor() {}

  /** Construct with given x and the parameters of the basis
   *
   * @param Args: Variadic template parameter for functor arguments.
   *
   * @param key: Factor key
   * @param z: Measurement object of type FUNCTOR::return_type
   * @param model: Noise model
   * @param args: Variable number of arguments used to instantiate functor
   */
  template <typename... Args>
  FunctorizedFactor(Key key, const typename FUNCTOR::return_type &z,
                    const SharedNoiseModel &model, Args &&... args)
      : Base(model, key), measured_(z), noiseModel_(model),
        func_(std::forward<Args>(args)...) {}

  virtual ~FunctorizedFactor() {}

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new FunctorizedFactor<FUNCTOR>(*this)));
  }

  Vector evaluateError(const T &params,
                       boost::optional<Matrix &> H = boost::none) const {
    typename FUNCTOR::return_type x = func_(params, H);
    Vector error = traits<typename FUNCTOR::return_type>::Local(measured_, x);
    return error;
  }

  /// @name Testable
  /// @{
  void print(const std::string &s = "",
             const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
    Base::print(s, keyFormatter);
    std::cout << s << (s != "" ? " " : "") << "FunctorizedFactor("
              << keyFormatter(this->key()) << ")" << std::endl;
    traits<typename FUNCTOR::return_type>::Print(measured_, "  measurement: ");
    std::cout << "  noise model sigmas: " << noiseModel_->sigmas().transpose()
              << std::endl;
  }

  virtual bool equals(const NonlinearFactor &other, double tol = 1e-9) const {
    const FunctorizedFactor<FUNCTOR> *e =
        dynamic_cast<const FunctorizedFactor<FUNCTOR> *>(&other);
    const bool base = Base::equals(*e, tol);
    return e && Base::equals(other, tol) &&
           traits<typename FUNCTOR::return_type>::Equals(this->measured_, e->measured_,
                                                tol);
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
template <typename FUNCTOR>
struct traits<FunctorizedFactor<FUNCTOR>>
    : public Testable<FunctorizedFactor<FUNCTOR>> {};

} // namespace gtsam
