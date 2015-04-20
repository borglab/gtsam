/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationBase.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationBase.h>

namespace gtsam {

class ImuFactorBase {

protected:

  Vector3 gravity_;
  Vector3 omegaCoriolis_;
  boost::optional<Pose3> body_P_sensor_; ///< The pose of the sensor in the body frame
  bool use2ndOrderCoriolis_; ///< Controls whether higher order terms are included when calculating the Coriolis Effect

public:

  /** Default constructor - only use for serialization */
  ImuFactorBase() :
      gravity_(Vector3(0.0, 0.0, 9.81)), omegaCoriolis_(Vector3(0.0, 0.0, 0.0)), body_P_sensor_(
          boost::none), use2ndOrderCoriolis_(false) {
  }

  /**
   *  Default constructor, stores basic quantities required by the Imu factors
   * @param gravity Gravity vector expressed in the global frame
   * @param omegaCoriolis Rotation rate of the global frame w.r.t. an inertial frame
   * @param body_P_sensor Optional pose of the sensor frame in the body frame
   * @param use2ndOrderCoriolis When true, the second-order term is used in the calculation of the Coriolis Effect
   */
  ImuFactorBase(const Vector3& gravity, const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none,
      const bool use2ndOrderCoriolis = false) :
      gravity_(gravity), omegaCoriolis_(omegaCoriolis), body_P_sensor_(
          body_P_sensor), use2ndOrderCoriolis_(use2ndOrderCoriolis) {
  }

  /// Methods to access class variables
  const Vector3& gravity() const {
    return gravity_;
  }
  const Vector3& omegaCoriolis() const {
    return omegaCoriolis_;
  }

  /// Needed for testable
  //------------------------------------------------------------------------------
  void print(const std::string& /*s*/) const {
    std::cout << "  gravity: [ " << gravity_.transpose() << " ]" << std::endl;
    std::cout << "  omegaCoriolis: [ " << omegaCoriolis_.transpose() << " ]"
        << std::endl;
    std::cout << "  use2ndOrderCoriolis: [ " << use2ndOrderCoriolis_ << " ]"
        << std::endl;
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
  }

  /// Needed for testable
  //------------------------------------------------------------------------------
  bool equals(const ImuFactorBase& expected, double tol) const {
    return equal_with_abs_tol(gravity_, expected.gravity_, tol)
        && equal_with_abs_tol(omegaCoriolis_, expected.omegaCoriolis_, tol)
        && (use2ndOrderCoriolis_ == expected.use2ndOrderCoriolis_)
        && ((!body_P_sensor_ && !expected.body_P_sensor_)
            || (body_P_sensor_ && expected.body_P_sensor_
                && body_P_sensor_->equals(*expected.body_P_sensor_)));
  }

};

} /// namespace gtsam
