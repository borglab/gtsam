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

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

  /**
   * Binary factor for a range measurement
   * @addtogroup SLAM
   */
  template<class POSE, class POINT>
  class RangeFactor: public NoiseModelFactor2<POSE, POINT> {
  private:

    double measured_; /** measurement */
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    typedef RangeFactor<POSE, POINT> This;
    typedef NoiseModelFactor2<POSE, POINT> Base;

    typedef POSE Pose;
    typedef POINT Point;

    // Concept requirements for this factor
    GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(POSE, POINT)

  public:

    RangeFactor() {} /* Default constructor */

    RangeFactor(Key poseKey, Key pointKey, double measured,
        const SharedNoiseModel& model, boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, poseKey, pointKey), measured_(measured), body_P_sensor_(body_P_sensor) {
    }

    virtual ~RangeFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** h(x)-z */
    Vector evaluateError(const POSE& pose, const POINT& point,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
      double hx;
      if(body_P_sensor_) {
        if(H1) {
          Matrix H0;
          hx = pose.compose(*body_P_sensor_, H0).range(point, H1, H2);
          *H1 = *H1 * H0;
        } else {
          hx = pose.compose(*body_P_sensor_).range(point, H1, H2);
        }
      } else {
        hx = pose.range(point, H1, H2);
      }
      return (Vector(1) << hx - measured_);
    }

    /** return the measured */
    double measured() const {
      return measured_;
    }

    /** equals specialized to this factor */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e = dynamic_cast<const This*> (&expected);
      return e != NULL
          && Base::equals(*e, tol)
          && fabs(this->measured_ - e->measured_) < tol
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /** print contents */
    void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "RangeFactor, range = " << measured_ << std::endl;
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
    }
  }; // RangeFactor

} // namespace gtsam
