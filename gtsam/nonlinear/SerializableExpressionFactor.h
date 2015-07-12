/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SerializableExpressionFactor.h
 * @date July 2015
 * @author Frank Dellaert
 * @brief ExpressionFactor variant that supports serialization
 */

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>

namespace gtsam {

/**
 * ExpressionFactor variant that supports serialization
 * Simply overload the pure virtual method [expression] to construct an
 * expression from keys_
 */
template <typename T>
class SerializableExpressionFactor : public ExpressionFactor<T> {
 public:
  /// Constructor takes only two arguments, still need to call initialize
  SerializableExpressionFactor(const SharedNoiseModel& noiseModel,
                               const T& measurement)
      : ExpressionFactor<T>(noiseModel, measurement) {}

  /// Destructor
  virtual ~SerializableExpressionFactor() {}

 protected:
  /// Return an expression that predicts the measurement given Values
  virtual Expression<T> expression() const = 0;

  /// Default constructor, for serialization
  SerializableExpressionFactor() {}

  /// Save to an archive: just saves the base class
  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    ar << boost::serialization::make_nvp(
        "ExpressionFactor",
        boost::serialization::base_object<ExpressionFactor<T> >(*this));
  }

  /// Load from an archive, creating a valid expression using the overloaded
  /// [expression] method
  template <class Archive>
  void load(Archive& ar, const unsigned int /*version*/) {
    ar >> boost::serialization::make_nvp(
              "ExpressionFactor",
              boost::serialization::base_object<ExpressionFactor<T> >(*this));
    this->initialize(expression());
  }

  // Indicate that we implement save/load separately, and be friendly to boost
  BOOST_SERIALIZATION_SPLIT_MEMBER()
  friend class boost::serialization::access;
};
// SerializableExpressionFactor

/**
 * Binary specialization of SerializableExpressionFactor
 * Enforces expression method with two keys, and provides evaluateError
 */
template <typename T, typename A1, typename A2>
class SerializableExpressionFactor2 : public SerializableExpressionFactor<T> {
 public:
  /// Constructor takes care of keys, but still need to call initialize
  SerializableExpressionFactor2(Key key1, Key key2,
                                const SharedNoiseModel& noiseModel,
                                const T& measurement)
      : SerializableExpressionFactor<T>(noiseModel, measurement) {
    this->keys_.push_back(key1);
    this->keys_.push_back(key2);
  }

  /// Destructor
  virtual ~SerializableExpressionFactor2() {}

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

  /// Return an expression that predicts the measurement given Values
  virtual Expression<T> expression(Key key1, Key key2) const = 0;

 protected:
  /// Default constructor, for serialization
  SerializableExpressionFactor2() {}

 private:
  /// Return an expression that predicts the measurement given Values
  virtual Expression<T> expression() const {
    return expression(this->keys_[0], this->keys_[1]);
  }
};
// SerializableExpressionFactor2

}  // \ namespace gtsam
