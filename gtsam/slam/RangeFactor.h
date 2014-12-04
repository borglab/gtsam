/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  RangeFactor.H
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/concepts.h>
#include <boost/lexical_cast.hpp>

namespace gtsam {

/**
 * Binary factor for a range measurement
 * @addtogroup SLAM
 */
template<class T1, class T2 = T1>
class RangeFactor: public NoiseModelFactor2<T1, T2> {
private:

  double measured_; /** measurement */

  typedef RangeFactor<T1, T2> This;
  typedef NoiseModelFactor2<T1, T2> Base;

  // Concept requirements for this factor
  GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(T1, T2)

public:

  RangeFactor() {
  } /* Default constructor */

  RangeFactor(Key key1, Key key2, double measured,
      const SharedNoiseModel& model) :
      Base(model, key1, key2), measured_(measured) {
  }

  virtual ~RangeFactor() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** h(x)-z */
  Vector evaluateError(const T1& t1, const T2& t2, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none) const {
    double hx;
    hx = t1.range(t2, H1, H2);
    return (Vector(1) << hx - measured_).finished();
  }

  /** return the measured */
  double measured() const {
    return measured_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol)
        && fabs(this->measured_ - e->measured_) < tol;
  }

  /** print contents */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "RangeFactor, range = " << measured_ << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
};
// RangeFactor

/**
 * Binary factor for a range measurement, with a transform applied
 * @addtogroup SLAM
 */
template<class POSE, class T2 = POSE>
class RangeFactorWithTransform: public NoiseModelFactor2<POSE, T2> {
private:

  double measured_; /** measurement */
  POSE body_P_sensor_; ///< The pose of the sensor in the body frame

  typedef RangeFactorWithTransform<POSE, T2> This;
  typedef NoiseModelFactor2<POSE, T2> Base;

  // Concept requirements for this factor
  GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(POSE, T2)

public:

  RangeFactorWithTransform() {
  } /* Default constructor */

  RangeFactorWithTransform(Key key1, Key key2, double measured,
      const SharedNoiseModel& model, const POSE& body_P_sensor) :
      Base(model, key1, key2), measured_(measured), body_P_sensor_(
          body_P_sensor) {
  }

  virtual ~RangeFactorWithTransform() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** h(x)-z */
  Vector evaluateError(const POSE& t1, const T2& t2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    double hx;
    if (H1) {
      Matrix H0;
      hx = t1.compose(body_P_sensor_, H0).range(t2, H1, H2);
      *H1 = *H1 * H0;
    } else {
      hx = t1.compose(body_P_sensor_).range(t2, H1, H2);
    }
    return (Vector(1) << hx - measured_).finished();
  }

  /** return the measured */
  double measured() const {
    return measured_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol)
        && fabs(this->measured_ - e->measured_) < tol
        && body_P_sensor_.equals(e->body_P_sensor_);
  }

  /** print contents */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "RangeFactor, range = " << measured_ << std::endl;
    this->body_P_sensor_.print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
};
// RangeFactor

}// namespace gtsam
