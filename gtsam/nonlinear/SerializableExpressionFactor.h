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
 * Simply overload the pure virtual method [expression] to construct an expression from keys_
 */
template <typename T>
class SerializableExpressionFactor : public ExpressionFactor<T> {
 public:
  /// Constructor takes only two arguments, still need to call initialize
  SerializableExpressionFactor(const SharedNoiseModel& noiseModel, const T& measurement)
      : ExpressionFactor<T>(noiseModel, measurement) {
  }

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
struct traits<SerializableExpressionFactor<T> >
    : public Testable<SerializableExpressionFactor<T> > {};

}// \ namespace gtsam

