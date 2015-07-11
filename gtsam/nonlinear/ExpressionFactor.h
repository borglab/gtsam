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

protected:

  typedef ExpressionFactor<T> This;
  static const int Dim = traits<T>::dimension;

  T measurement_;  ///< the measurement to be compared with the expression
  Expression<T> expression_;  ///< the expression that is AD enabled
  FastVector<int> dims_;      ///< dimensions of the Jacobian matrices

public:

  typedef boost::shared_ptr<ExpressionFactor<T> > shared_ptr;

  /// Constructor
  ExpressionFactor(const SharedNoiseModel& noiseModel,  //
                   const T& measurement, const Expression<T>& expression)
      : NoiseModelFactor(noiseModel), measurement_(measurement) {
    initialize(expression);
  }

  /// print relies on Testable traits being defined for T
  void print(const std::string& s, const KeyFormatter& keyFormatter) const {
    NoiseModelFactor::print(s, keyFormatter);
    traits<T>::Print(measurement_, s + ".measurement_");
  }

  /// equals relies on Testable traits being defined for T
  bool equals(const NonlinearFactor& f, double tol) const {
    const ExpressionFactor* p = dynamic_cast<const ExpressionFactor*>(&f);
    return p && NoiseModelFactor::equals(f, tol) &&
           traits<T>::Equals(measurement_, p->measurement_, tol) &&
           dims_ == p->dims_;
  }

  /**
   * Error function *without* the NoiseModel, \f$ h(x)-z \f$.
   * We override this method to provide
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if (H) {
      const T value = expression_.valueAndDerivatives(x, keys_, dims_, *H);
      return traits<T>::Local(measurement_, value);
    } else {
      const T value = expression_.value(x);
      return traits<T>::Local(measurement_, value);
    }
  }

  virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x) const {
    // Only linearize if the factor is active
    if (!active(x))
      return boost::shared_ptr<JacobianFactor>();

    // In case noise model is constrained, we need to provide a noise model
    SharedDiagonal noiseModel;
    if (noiseModel_->isConstrained()) {
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
    Ab(size()).col(0) = -traits<T>::Local(measurement_, value);

    // Whiten the corresponding system, Ab already contains RHS
    Vector b = Ab(size()).col(0); // need b to be valid for Robust noise models
    noiseModel_->WhitenSystem(Ab.matrix(), b);

    return factor;
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

protected:
 /// Default constructor, for serialization
 ExpressionFactor() {}

 /// Constructor for use by SerializableExpressionFactor
 ExpressionFactor(const SharedNoiseModel& noiseModel, const T& measurement)
     : NoiseModelFactor(noiseModel), measurement_(measurement) {
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
   boost::tie(keys_, dims_) = expression_.keysAndDims();
 }

private:
 /// Serialization function
 template <class ARCHIVE>
 void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
   ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(NoiseModelFactor);
   ar& boost::serialization::make_nvp("measurement_", this->measurement_);
 }

 friend class boost::serialization::access;
};
// ExpressionFactor


/**
 * ExpressionFactor variant that supports serialization
 * Simply overload the pure virtual method [expression] to construct an expression from keys_
 */
template <typename T>
class SerializableExpressionFactor : public ExpressionFactor<T> {
 public:
  /// Constructor takes only two arguments, still need to call initialize
  SerializableExpressionFactor(const SharedNoiseModel& noiseModel, const T& measurement)
      : ExpressionFactor<T>(noiseModel, measurement) {
  }

 protected:

  /// Return an expression that predicts the measurement given Values
  virtual Expression<T> expression() const = 0;

  /// Default constructor, for serialization
  SerializableExpressionFactor() {}

  /// Save to an archive: just saves the base class
  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    ar << boost::serialization::make_nvp(
              "ExpressionFactor", boost::serialization::base_object<ExpressionFactor<T> >(*this));
  }

  /// Load from an archive, creating a valid expression using the overloaded [expression] method
  template <class Archive>
  void load(Archive& ar, const unsigned int /*version*/) {
    ar >> boost::serialization::make_nvp(
              "ExpressionFactor", boost::serialization::base_object<ExpressionFactor<T> >(*this));
    this->initialize(expression());
  }

  // Indicate that we implement save/load separately, and be friendly to boost
  BOOST_SERIALIZATION_SPLIT_MEMBER()
  friend class boost::serialization::access;
};
// SerializableExpressionFactor

/// traits
template <typename T>
struct traits<ExpressionFactor<T> > : public Testable<ExpressionFactor<T> > {};

}// \ namespace gtsam

