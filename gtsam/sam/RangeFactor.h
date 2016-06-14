/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  RangeFactor.h
 *  @brief Serializable factor induced by a range measurement
 *  @date July 2015
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>

namespace gtsam {

// forward declaration of Range functor, assumed partially specified
template <typename A1, typename A2>
struct Range;

/**
 * Binary factor for a range measurement
 * Works for any two types A1,A2 for which the functor Range<A1,A2>() is defined
 * @addtogroup SAM
 */
template <typename A1, typename A2 = A1, typename T = double>
class RangeFactor : public ExpressionFactor2<T, A1, A2> {
 private:
  typedef RangeFactor<A1, A2> This;
  typedef ExpressionFactor2<T, A1, A2> Base;

 public:
  /// default constructor
  RangeFactor() {}

  RangeFactor(Key key1, Key key2, T measured, const SharedNoiseModel& model)
      : Base(key1, key2, model, measured) {
    this->initialize(expression(key1, key2));
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  virtual Expression<T> expression(Key key1, Key key2) const {
    Expression<A1> a1_(key1);
    Expression<A2> a2_(key2);
    return Expression<T>(Range<A1, A2>(), a1_, a2_);
  }

  /// print
  void print(const std::string& s = "",
             const KeyFormatter& kf = DefaultKeyFormatter) const {
    std::cout << s << "RangeFactor" << std::endl;
    Base::print(s, kf);
  }
};  // \ RangeFactor

/// traits
template <typename A1, typename A2, typename T>
struct traits<RangeFactor<A1, A2, T> >
    : public Testable<RangeFactor<A1, A2, T> > {};

/**
 * Binary factor for a range measurement, with a transform applied
 * @addtogroup SAM
 */
template <typename A1, typename A2 = A1,
          typename T = typename Range<A1, A2>::result_type>
class RangeFactorWithTransform : public ExpressionFactor2<T, A1, A2> {
 private:
  typedef RangeFactorWithTransform<A1, A2> This;
  typedef ExpressionFactor2<T, A1, A2> Base;

  A1 body_T_sensor_;  ///< The pose of the sensor in the body frame

 public:
  //// Default constructor
  RangeFactorWithTransform() {}

  RangeFactorWithTransform(Key key1, Key key2, T measured,
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
  virtual Expression<T> expression(Key key1, Key key2) const {
    Expression<A1> body_T_sensor__(body_T_sensor_);
    Expression<A1> nav_T_body_(key1);
    Expression<A1> nav_T_sensor_(traits<A1>::Compose, nav_T_body_,
                                 body_T_sensor__);
    Expression<A2> a2_(key2);
    return Expression<T>(Range<A1, A2>(), nav_T_sensor_, a2_);
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
  template <typename ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
  }
};  // \ RangeFactorWithTransform

/// traits
template <typename A1, typename A2, typename T>
struct traits<RangeFactorWithTransform<A1, A2, T> >
    : public Testable<RangeFactorWithTransform<A1, A2, T> > {};

}  // \ namespace gtsam
