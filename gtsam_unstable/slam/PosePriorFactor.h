/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PosePriorFactor.h
 *  @author Frank Dellaert
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

  /**
   * A class for a soft prior on any Value type
   * @addtogroup SLAM
   */
  template<class POSE>
  class PosePriorFactor: public NoiseModelFactor1<POSE> {

  private:

    typedef PosePriorFactor<POSE> This;
    typedef NoiseModelFactor1<POSE> Base;

    POSE prior_; /** The measurement */
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    /** concept check by type */
    GTSAM_CONCEPT_TESTABLE_TYPE(POSE)
    GTSAM_CONCEPT_POSE_TYPE(POSE)
  public:

    /// shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<PosePriorFactor<POSE> > shared_ptr;

    /** default constructor - only use for serialization */
    PosePriorFactor() {}

    virtual ~PosePriorFactor() {}

    /** Constructor */
    PosePriorFactor(Key key, const POSE& prior, const SharedNoiseModel& model,
        boost::optional<POSE> body_P_sensor = boost::none) :
      Base(model, key), prior_(prior), body_P_sensor_(body_P_sensor) {
    }

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "PriorFactor on " << keyFormatter(this->key()) << "\n";
      prior_.print("  prior mean: ");
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This* e = dynamic_cast<const This*> (&expected);
      return e != nullptr
          && Base::equals(*e, tol)
          && this->prior_.equals(e->prior_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const POSE& p, boost::optional<Matrix&> H = boost::none) const {
      if(body_P_sensor_) {
        // manifold equivalent of h(x)-z -> log(z,h(x))
        return prior_.localCoordinates(p.compose(*body_P_sensor_, H));
      } else {
        if(H) (*H) = I_6x6;
        // manifold equivalent of h(x)-z -> log(z,h(x))
        return prior_.localCoordinates(p);
      }
    }

    const POSE& prior() const { return prior_; }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(prior_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
    }
  };

} /// namespace gtsam
