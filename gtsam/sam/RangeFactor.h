/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  RangeFactor.h
 *  @brief Serializable factor induced by a range measurement between two points
 *and/or poses
 *  @date July 2015
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/SerializableExpressionFactor.h>
#include <gtsam/geometry/concepts.h>
#include <boost/lexical_cast.hpp>

namespace gtsam {

/**
 * Binary factor for a range measurement
 * @addtogroup SAM
 */
template <class A1, class A2 = A1>
struct RangeFactor : public SerializableExpressionFactor2<double, A1, A2> {
 private:
  typedef RangeFactor<A1, A2> This;
  typedef SerializableExpressionFactor2<double, A1, A2> Base;

  // Concept requirements for this factor
  GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(A1, A2)

 public:
  /// default constructor
  RangeFactor() {}

  RangeFactor(Key key1, Key key2, double measured,
              const SharedNoiseModel& model)
      : Base(key1, key2, model, measured) {
    this->initialize(expression(key1, key2));
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  virtual Expression<double> expression(Key key1, Key key2) const {
    Expression<A1> t1_(key1);
    Expression<A2> t2_(key2);
    return Expression<double>(t1_, &A1::range, t2_);
  }

  /// print
  void print(const std::string& s = "",
             const KeyFormatter& kf = DefaultKeyFormatter) const {
    std::cout << s << "RangeFactor" << std::endl;
    Base::print(s, kf);
  }
};  // \ RangeFactor

/// traits
template <class A1, class A2>
struct traits<RangeFactor<A1, A2> > : public Testable<RangeFactor<A1, A2> > {};

/**
 * Binary factor for a range measurement, with a transform applied
 * @addtogroup SAM
 */
template <class A1, class A2 = A1>
class RangeFactorWithTransform
    : public SerializableExpressionFactor2<double, A1, A2> {
 private:
  typedef RangeFactorWithTransform<A1, A2> This;
  typedef SerializableExpressionFactor2<double, A1, A2> Base;

  A1 body_T_sensor_;  ///< The pose of the sensor in the body frame

  // Concept requirements for this factor
  GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(A1, A2)

 public:
  //// Default constructor
  RangeFactorWithTransform() {}

  RangeFactorWithTransform(Key key1, Key key2, double measured,
                           const SharedNoiseModel& model,
                           const A1& body_T_sensor)
      : Base(key1, key2, model, measured), body_T_sensor_(body_T_sensor) {
    this->initialize(expression(key1, key2));
  }

  virtual ~RangeFactorWithTransform() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  virtual Expression<double> expression(Key key1, Key key2) const {
    Expression<A1> body_T_sensor__(body_T_sensor_);
    Expression<A1> nav_T_body_(key1);
    Expression<A1> nav_T_sensor_(traits<A1>::Compose, nav_T_body_,
                                 body_T_sensor__);
    Expression<A2> t2_(key2);
    return Expression<double>(nav_T_sensor_, &A1::range, t2_);
  }

  /** print contents */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "RangeFactorWithTransform" << std::endl;
    this->body_T_sensor_.print("  sensor pose in body frame: ");
    Base::print(s, keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
  }
};  // \ RangeFactorWithTransform

/// traits
template <class A1, class A2>
struct traits<RangeFactorWithTransform<A1, A2> >
    : public Testable<RangeFactorWithTransform<A1, A2> > {};

}  // \ namespace gtsam
