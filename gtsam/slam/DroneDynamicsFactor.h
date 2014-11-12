

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file DroneDynamicsFactor.h
 * @author Duy-Nguyen Ta
 * @date Sep 29, 2014
 */
// Implementation is incorrect use DroneDynamicsVelXYFactor instead.
#pragma once

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

  /**
   * Binary factor for a range measurement
   * @addtogroup SLAM
   */
  class DroneDynamicsFactor: public NoiseModelFactor2<Pose3, LieVector> {
  private:

    LieVector measured_; /** body velocity measured from raw acc and motor inputs*/

    typedef DroneDynamicsFactor This;
    typedef NoiseModelFactor2<Pose3, LieVector> Base;

  public:

    DroneDynamicsFactor() {} /* Default constructor */

    DroneDynamicsFactor(Key poseKey, Key velKey, const LieVector& measured,
        const SharedNoiseModel& model) :
          Base(model, poseKey, velKey), measured_(measured) {
    }

    virtual ~DroneDynamicsFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** h(x)-z */
    Vector evaluateError(const Pose3& pose, const LieVector& vel,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {

      // error = v - wRb*measured
      Rot3 wRb = pose.rotation();
      Vector3 error;

      if (H1 || H2) {
        *H2 = eye(3);
        *H1 = zeros(3,6);
        Matrix H1Rot;
        error = wRb.unrotate(Point3(vel.vector()), H1Rot, H2).vector() - measured_.vector();
        (*H1).block(0,0,3,3) = H1Rot;
      }
      else {
        error = wRb.unrotate(Point3(vel.vector())).vector() - measured_.vector();
      }

      return error;
    }

    /** return the measured */
    LieVector measured() const {
      return measured_;
    }

    /** equals specialized to this factor */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e = dynamic_cast<const This*> (&expected);
      return e != NULL
          && Base::equals(*e, tol)
          ;
    }

    /** print contents */
    void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "DroneDynamicsFactor, measured = " << measured_.vector().transpose() << std::endl;
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
    }
  }; // DroneDynamicsFactor

} // namespace gtsam



