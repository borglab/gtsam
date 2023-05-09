/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BiasedGPSFactor.h
 *  @author Luca Carlone
 **/
#pragma once

#include <ostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

  /**
   * A class to model GPS measurements, including a bias term which models
   * common-mode errors and that can be partially corrected if other sensors are used
   * @addtogroup SLAM
   */
  class BiasedGPSFactor: public NoiseModelFactor2<Pose3, Point3> {

  private:

    typedef BiasedGPSFactor This;
    typedef NoiseModelFactor2<Pose3, Point3> Base;

    Point3 measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<BiasedGPSFactor> shared_ptr;

    /** default constructor - only use for serialization */
    BiasedGPSFactor() {}

    /** Constructor */
    BiasedGPSFactor(Key posekey, Key biaskey, const Point3 measured,
        const SharedNoiseModel& model) :
      Base(model, posekey, biaskey), measured_(measured) {
    }

    ~BiasedGPSFactor() override {}

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "BiasedGPSFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n"
          << "  measured: " << measured_.transpose() << std::endl;
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(this->measured_, e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const Pose3& pose, const Point3& bias,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
            boost::none) const override {

      if (H1 || H2){
        H1->resize(3,6); // jacobian wrt pose
        (*H1) << Matrix3::Zero(),  pose.rotation().matrix();
        H2->resize(3,3); // jacobian wrt bias
        (*H2) << Matrix3::Identity();
      }
      return pose.translation() + bias - measured_;
    }

    /** return the measured */
    const Point3 measured() const {
      return measured_;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // \class BiasedGPSFactor

} /// namespace gtsam
