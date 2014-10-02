

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DroneDynamicsVelXYFactor.h
 *
 *  Created on: Oct 1, 2014
 *      Author: krunal
 */
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
  class DroneDynamicsVelXYFactor: public NoiseModelFactor3<Pose3, LieVector, LieVector> {
  private:

    Vector motors_; /** motor inputs */
    Vector acc_;    /** raw acc */
    Matrix M_;

    typedef DroneDynamicsVelXYFactor This;
    typedef NoiseModelFactor3<Pose3, LieVector, LieVector> Base;

  public:

    DroneDynamicsVelXYFactor() {} /* Default constructor */

    DroneDynamicsVelXYFactor(Key poseKey, Key velKey, Key cKey, const Vector& motors, const Vector& acc,
        const SharedNoiseModel& model) :
          Base(model, poseKey, velKey, cKey), motors_(motors), acc_(acc), M_(computeM(motors, acc)) {
    }

    virtual ~DroneDynamicsVelXYFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    // M = [sum(sqrt(m))ax 1 0 0; 0 0 sum(sqrt(m))ay 1; 0 0 0 0]
    Matrix computeM(const Vector& motors, const Vector& acc) const {
      Matrix M = zeros(3,4);
      double sqrtSumMotors = sqrt(motors(0)) + sqrt(motors(1)) + sqrt(motors(2)) + sqrt(motors(3));
      M(0,0) = sqrtSumMotors*acc(0); M(0, 1) = 1.0;
      M(1,2) = 1.0; M(1, 3) = sqrtSumMotors*acc(1);
      return M;
    }

    /** h(x)-z */
    Vector evaluateError(const Pose3& pose, const LieVector& vel, const LieVector& c,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none) const {

      // error = R'*v - M*c, where
      Rot3 wRb = pose.rotation();
      Vector error;

      if (H1 || H2 || H3) {
        *H1 = zeros(3, 6);
        *H2 = eye(3);
        Matrix H1Rot;
        error = wRb.unrotate(Point3(vel.vector()), H1Rot, H2).vector() - M_*c.vector();
        (*H1).block(0,0,3,3) = H1Rot;

        *H3 = -M_;
      }
      else {
        error = wRb.unrotate(Point3(vel.vector())).vector() - M_*c.vector();
      }

      return error;
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
      std::cout << s << "DroneDynamicsVelXYFactor, motors = " << motors_.transpose() << " acc: " << acc_.transpose() << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(motors_);
      ar & BOOST_SERIALIZATION_NVP(acc_);
    }
  }; // DroneDynamicsVelXYFactor

} // namespace gtsam



