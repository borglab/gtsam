/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Expression.h
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief Expressions for Block Automatic Differentiation
 */

#pragma once

#include <array>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <numeric>

namespace gtsam {

/**
 * Factor that supports arbitrary expressions via AD.
 *
 * Arbitrary instances of this template can be directly inserted into a factor
 * graph for optimization. However, to enable the correct (de)serialization of
 * such instances, the user should declare derived classes from this template,
 * implementing expresion(), serialize(), clone(), print(), and defining the
 * corresponding `struct traits<NewFactor> : public Testable<NewFactor> {}`.
 *
 * \tparam T Type for measurements.
 *
 */
template <typename T>
class ExpressionFactor : public NoiseModelFactor {
  BOOST_CONCEPT_ASSERT((IsTestable<T>));

protected:

  typedef ExpressionFactor<T> This;
  static const int Dim = traits<T>::dimension;

  T measured_;  ///< the measurement to be compared with the expression
  Expression<T> expression_;  ///< the expression that is AD enabled
  FastVector<int> dims_;      ///< dimensions of the Jacobian matrices


 public:

  // Provide access to the Matrix& version of unwhitenedError:
  using NoiseModelFactor::unwhitenedError;
  typedef std::shared_ptr<ExpressionFactor<T> > shared_ptr;

  /**
   * Constructor: creates a factor from a measurement and measurement function
   *   @param noiseModel the noise model associated with a measurement
   *   @param measurement actual value of the measurement, of type T
   *   @param expression predicts the measurement from Values
   * The keys associated with the factor, returned by keys(), are sorted.
   */
  ExpressionFactor(const SharedNoiseModel& noiseModel,  //
                   const T& measurement, const Expression<T>& expression)
      : NoiseModelFactor(noiseModel), measured_(measurement) {
    initialize(expression);
  }

  /// Destructor
  ~ExpressionFactor() override {}

  /** return the measurement */
  const T& measured() const { return measured_; }

  /// print relies on Testable traits being defined for T
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    NoiseModelFactor::print(s, keyFormatter);
    traits<T>::Print(measured_, "ExpressionFactor with measurement: ");
  }

  /// equals relies on Testable traits being defined for T
  bool equals(const NonlinearFactor& f, double tol) const override {
    const ExpressionFactor* p = dynamic_cast<const ExpressionFactor*>(&f);
    return p && NoiseModelFactor::equals(f, tol) &&
           traits<T>::Equals(measured_, p->measured_, tol) &&
           dims_ == p->dims_;
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) -> Local(h(x),z) \f$.
   * We override this method to provide
   * both the function evaluation and its derivative(s) in H.
   */
  Vector unwhitenedError(const Values& x,
    OptionalMatrixVecType H = nullptr) const override {
    if (H) {
      const T value = expression_.valueAndDerivatives(x, keys_, dims_, *H);
      // NOTE(hayk): Doing the reverse, AKA Local(measured_, value) is not correct here
      // because it would use the tangent space of the measurement instead of the value.
      return -traits<T>::Local(value, measured_);
    } else {
      const T value = expression_.value(x);
      return -traits<T>::Local(value, measured_);
    }
  }

  std::shared_ptr<GaussianFactor> linearize(const Values& x) const override {
    // Only linearize if the factor is active
    if (!active(x))
      return std::shared_ptr<JacobianFactor>();

    // In case noise model is constrained, we need to provide a noise model
    SharedDiagonal noiseModel;
    if (noiseModel_ && noiseModel_->isConstrained()) {
      noiseModel = std::static_pointer_cast<noiseModel::Constrained>(
          noiseModel_)->unit();
    }

    // Create a writeable JacobianFactor in advance
    std::shared_ptr<JacobianFactor> factor(
        new JacobianFactor(keys_, dims_, Dim, noiseModel));

    // Wrap keys and VerticalBlockMatrix into structure passed to expression_
    VerticalBlockMatrix& Ab = factor->matrixObject();
    internal::JacobianMap jacobianMap(keys_, Ab);

    // Zero out Jacobian so we can simply add to it
    Ab.matrix().setZero();

    // Get value and Jacobians, writing directly into JacobianFactor
    T value = expression_.valueAndJacobianMap(x, jacobianMap); // <<< Reverse AD happens here !

    // Evaluate error and set RHS vector b
    Ab(size()).col(0) = traits<T>::Local(value, measured_);

    // Whiten the corresponding system, Ab already contains RHS
    if (noiseModel_) {
      Vector b = Ab(size()).col(0);  // need b to be valid for Robust noise models
      noiseModel_->WhitenSystem(Ab.matrix(), b);
    }

    return factor;
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

protected:
 ExpressionFactor() {}
 /// Default constructor, for serialization

 /// Constructor for serializable derived classes
 ExpressionFactor(const SharedNoiseModel& noiseModel, const T& measurement)
     : NoiseModelFactor(noiseModel), measured_(measurement) {
   // Not properly initialized yet, need to call initialize
 }

 /// Initialize with constructor arguments
 void initialize(const Expression<T>& expression) {
   if (!noiseModel_)
     throw std::invalid_argument("ExpressionFactor: no NoiseModel.");
   if (noiseModel_->dim() != Dim)
     throw std::invalid_argument(
         "ExpressionFactor was created with a NoiseModel of incorrect dimension.");
   expression_ = expression;

   // Get keys and dimensions for Jacobian matrices
   // An Expression is assumed unmutable, so we do this now
   if (keys_.empty()) {
     // This is the case when called in ExpressionFactor Constructor.
     // We then take the keys from the expression in sorted order.
     std::tie(keys_, dims_) = expression_.keysAndDims();
   } else {
     // This happens with classes derived from BinaryExpressionFactor etc.
     // In that case, the keys_ are already defined and we just need to grab
     // the dimensions in the correct order.
     std::map<Key, int> keyedDims;
     expression_.dims(keyedDims);
     for (Key key : keys_) dims_.push_back(keyedDims[key]);
   }
 }

 /// Recreate expression from keys_ and measured_, used in load below.
 /// Needed to deserialize a derived factor
 virtual Expression<T> expression() const {
   throw std::runtime_error("ExpressionFactor::expression not provided: cannot deserialize.");
 }

private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
 /// Save to an archive: just saves the base class
 template <class Archive>
 void save(Archive& ar, const unsigned int /*version*/) const {
   ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(NoiseModelFactor);
   ar << boost::serialization::make_nvp("measured_", this->measured_);
 }

 /// Load from an archive, creating a valid expression using the overloaded
 /// [expression] method
 template <class Archive>
 void load(Archive& ar, const unsigned int /*version*/) {
   ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(NoiseModelFactor);
   ar >> boost::serialization::make_nvp("measured_", this->measured_);
   this->initialize(expression());
 }

 // Indicate that we implement save/load separately, and be friendly to boost
 BOOST_SERIALIZATION_SPLIT_MEMBER()

 friend class boost::serialization::access;
#endif

 // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
 enum { NeedsToAlign = (sizeof(T) % 16) == 0 };
  public:
	  GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};
// ExpressionFactor

/// traits
template <typename T>
struct traits<ExpressionFactor<T> > : public Testable<ExpressionFactor<T> > {};

/**
 * N-ary variadic template for ExpressionFactor meant as a base class for N-ary
 * factors. Enforces an 'expression' method with N keys.
 * Derived class (an N-factor!) needs to call 'initialize'.
 *
 * Does not provide backward compatible 'evaluateError'.
 *
 * \tparam T Type for measurements. The rest of template arguments are types
 *         for the N key-indexed Values.
 *
 */
template <typename T, typename... Args>
class ExpressionFactorN : public ExpressionFactor<T> {
public:
  static const std::size_t NARY_EXPRESSION_SIZE = sizeof...(Args);
  using ArrayNKeys = std::array<Key, NARY_EXPRESSION_SIZE>;
    
  // Provide access to the Matrix& version of unwhitenedError:
  using ExpressionFactor<T>::unwhitenedError;

  /// Destructor
  ~ExpressionFactorN() override = default;

  // Don't provide backward compatible evaluateVector(), due to its problematic
  // variable length of optional Jacobian arguments. Vector evaluateError(const
  // Args... args,...);

  /// Recreate expression from given keys_ and measured_, used in load
  /// Needed to deserialize a derived factor
  virtual Expression<T> expression(const ArrayNKeys &keys) const {
    throw std::runtime_error(
        "ExpressionFactorN::expression not provided: cannot deserialize.");
  }

protected:
  /// Default constructor, for serialization
  ExpressionFactorN() = default;

  /// Constructor takes care of keys, but still need to call initialize
  ExpressionFactorN(const ArrayNKeys &keys, const SharedNoiseModel &noiseModel,
                    const T &measurement)
      : ExpressionFactor<T>(noiseModel, measurement) {
    for (const auto &key : keys)
      Factor::keys_.push_back(key);
  }

private:
  /// Return an expression that predicts the measurement given Values
  Expression<T> expression() const override {
    ArrayNKeys keys;
    int idx = 0;
    for (const auto &key : Factor::keys_)
      keys[idx++] = key;
    return expression(keys);
  }

  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &boost::serialization::make_nvp(
        "ExpressionFactorN",
        boost::serialization::base_object<ExpressionFactor<T>>(*this));
  }
};
/// traits
template <typename T, typename... Args>
struct traits<ExpressionFactorN<T, Args...>>
    : public Testable<ExpressionFactorN<T, Args...>> {};
// ExpressionFactorN

} // namespace gtsam
