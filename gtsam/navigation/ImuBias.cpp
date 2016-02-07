/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ImuBias.cpp
 * @date  Feb 2, 2012
 * @author Vadim Indelman, Stephen Williams
 */

#include "ImuBias.h"

#include <gtsam/geometry/Point3.h>
#include <iostream>

namespace gtsam {

/// All bias models live in the imuBias namespace
namespace imuBias {

/*
 * NOTES:
 * - Earth-rate correction:
 *     + Currently the user should supply R_ECEF_to_G, which is the rotation from ECEF to Local-Level system (NED or ENU as defined by the user).
 *     + R_ECEF_to_G can be calculated by approximated values of latitude and longitude of the system.
 *     + A relatively small distance is traveled w.r.t. to initial pose is assumed, since R_ECEF_to_G is constant.
 *        Otherwise, R_ECEF_to_G should be updated each time using the current lat-lon.
 *
 *  - Currently, an empty constructed is not enabled so that the user is forced to specify R_ECEF_to_G.
 */
//    // H1: Jacobian w.r.t. IMUBias
//    // H2: Jacobian w.r.t. pose
//    Vector CorrectGyroWithEarthRotRate(Vector measurement, const Pose3& pose, const Vector& w_earth_rate_G,
//        boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const {
//
//      Matrix R_G_to_I( pose.rotation().matrix().transpose() );
//      Vector w_earth_rate_I = R_G_to_I * w_earth_rate_G;
//
//      if (H1){
//        Matrix zeros3_3(zeros(3,3));
//        Matrix m_eye3(-eye(3));
//
//        *H1 = collect(2, &zeros3_3, &m_eye3);
//      }
//
//      if (H2){
//        Matrix zeros3_3(zeros(3,3));
//        Matrix H = -skewSymmetric(w_earth_rate_I);
//
//        *H2 = collect(2, &H, &zeros3_3);
//      }
//
//      //TODO: Make sure H2 is correct.
//
//      return measurement - biasGyro_ - w_earth_rate_I;
//
////      Vector bias_gyro_temp((Vector(3) << -bias_gyro_(0), bias_gyro_(1), bias_gyro_(2)));
////      return measurement - bias_gyro_temp - R_G_to_I * w_earth_rate_G;
//    }
/// ostream operator
std::ostream& operator<<(std::ostream& os, const ConstantBias& bias) {
  os << "acc = " << Point3(bias.accelerometer());
  os << " gyro = " << Point3(bias.gyroscope());
  return os;
}

/// print with optional string
void ConstantBias::print(const std::string& s) const {
  std::cout << s << *this << std::endl;
}

} // namespace imuBias

} // namespace gtsam

