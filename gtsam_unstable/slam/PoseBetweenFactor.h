/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BetweenFactor.h
 *  @author Frank Dellaert, Viorela Ila
 **/
#pragma once

#include <ostream>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam POSE the Pose type
   * @ingroup slam
   */
  template<class POSE>
  class PoseBetweenFactor: public NoiseModelFactorN<POSE, POSE> {

  private:

    typedef PoseBetweenFactor<POSE> This;
    typedef NoiseModelFactorN<POSE, POSE> Base;

    POSE measured_; /** The measurement */
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    /** concept check by type */
    GTSAM_CONCEPT_TESTABLE_TYPE(POSE)
    GTSAM_CONCEPT_POSE_TYPE(POSE)
  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<PoseBetweenFactor> shared_ptr;

    /** default constructor - only use for serialization */
    PoseBetweenFactor() {}

    /** Constructor */
    PoseBetweenFactor(Key key1, Key key2, const POSE& measured,
        const SharedNoiseModel& model, boost::optional<POSE> body_P_sensor = boost::none) :
      Base(model, key1, key2), measured_(measured), body_P_sensor_(body_P_sensor) {
    }

    ~PoseBetweenFactor() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "BetweenFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      measured_.print("  measured: ");
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != nullptr
          && Base::equals(*e, tol)
          && this->measured_.equals(e->measured_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const POSE& p1, const POSE& p2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const override {
      if(body_P_sensor_) {
        POSE hx;
        if(H1 || H2) {
          Matrix H3;
          hx = p1.compose(*body_P_sensor_,H3).between(p2.compose(*body_P_sensor_), H1, H2); // h(x)
          if(H1) (*H1) *= H3;
          if(H2) (*H2) *= H3;
        } else {
          hx = p1.compose(*body_P_sensor_).between(p2.compose(*body_P_sensor_)); // h(x)
        }
        // manifold equivalent of h(x)-z -> log(z,h(x))
        return measured_.localCoordinates(hx);
      } else {
        POSE hx = p1.between(p2, H1, H2); // h(x)
        // manifold equivalent of h(x)-z -> log(z,h(x))
        return measured_.localCoordinates(hx);
      }
    }

    /** return the measured */
    const POSE& measured() const {
      return measured_;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);

    }
  }; // \class PoseBetweenFactor

} /// namespace gtsam
