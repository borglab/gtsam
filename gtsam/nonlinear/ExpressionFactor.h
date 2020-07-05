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

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <numeric>

namespace gtsam {

/**

 * Factor that supports arbitrary expressions via AD
 */
template<typename T>
class ExpressionFactor: public NoiseModelFactor {
  BOOST_CONCEPT_ASSERT((IsTestable<T>));

protected:

  typedef ExpressionFactor<T> This;
  static const int Dim = traits<T>::dimension;

  T measured_;  ///< the measurement to be compared with the expression
  Expression<T> expression_;  ///< the expression that is AD enabled
  FastVector<int> dims_;      ///< dimensions of the Jacobian matrices


 public:
  typedef boost::shared_ptr<ExpressionFactor<T> > shared_ptr;

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
  virtual ~ExpressionFactor() {}

  /** return the measurement */
  const T& measured() const { return measured_; }

  /// print relies on Testable traits being defined for T
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    NoiseModelFactor::print(s, keyFormatter);
    traits<T>::Print(measured_, "ExpressionFactor with measurement: ");
  }

  /// equals relies on Testable traits being defined for T
  bool equals(const NonlinearFactor& f, double tol) const {
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
  virtual Vector unwhitenedError(const Values& x,
                                 boost::optional<std::vector<Matrix>&> H = boost::none) const {
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

  virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x) const {
    // Only linearize if the factor is active
    if (!active(x))
      return boost::shared_ptr<JacobianFactor>();

    // In case noise model is constrained, we need to provide a noise model
    SharedDiagonal noiseModel;
    if (noiseModel_ && noiseModel_->isConstrained()) {
      noiseModel = boost::static_pointer_cast<noiseModel::Constrained>(
          noiseModel_)->unit();
    }

    // Create a writeable JacobianFactor in advance
    boost::shared_ptr<JacobianFactor> factor(
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
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
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
     boost::tie(keys_, dims_) = expression_.keysAndDims();
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
 * Binary specialization of ExpressionFactor meant as a base class for binary
 * factors. Enforces an 'expression' method with two keys, and provides 'evaluateError'.
 * Derived class (a binary factor!) needs to call 'initialize'.
 */
template <typename T, typename A1, typename A2>
class ExpressionFactor2 : public ExpressionFactor<T> {
 public:
  /// Destructor
  virtual ~ExpressionFactor2() {}

  /// Backwards compatible evaluateError, to make existing tests compile
  Vector evaluateError(const A1& a1, const A2& a2,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const {
    Values values;
    values.insert(this->keys_[0], a1);
    values.insert(this->keys_[1], a2);
    std::vector<Matrix> H(2);
    Vector error = this->unwhitenedError(values, H);
    if (H1) (*H1) = H[0];
    if (H2) (*H2) = H[1];
    return error;
  }

  /// Recreate expression from given keys_ and measured_, used in load
  /// Needed to deserialize a derived factor
  virtual Expression<T> expression(Key key1, Key key2) const {
    throw std::runtime_error("ExpressionFactor2::expression not provided: cannot deserialize.");
  }

 protected:
  /// Default constructor, for serialization
  ExpressionFactor2() {}

  /// Constructor takes care of keys, but still need to call initialize
  ExpressionFactor2(Key key1, Key key2,
                                const SharedNoiseModel& noiseModel,
                                const T& measurement)
      : ExpressionFactor<T>(noiseModel, measurement) {
    this->keys_.push_back(key1);
    this->keys_.push_back(key2);
  }

 private:
  /// Return an expression that predicts the measurement given Values
  virtual Expression<T> expression() const {
    return expression(this->keys_[0], this->keys_[1]);
  }

  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "ExpressionFactor", boost::serialization::base_object<ExpressionFactor<T> >(*this));
  }
};
// ExpressionFactor2

}// \ namespace gtsam
