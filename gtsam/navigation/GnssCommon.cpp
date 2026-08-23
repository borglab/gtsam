/**
 *  @file   GnssCommon.cpp
 *  @brief  Implementation of shared GNSS utilities.
 *  @date   April 16, 2026
 **/

#include "GnssCommon.h"

#include <gtsam/base/Testable.h>

#include <limits>

namespace gtsam {
namespace gnss {

double geodist(const Point3& sat, const Point3& rcv, Point3& e,
               OptionalJacobian<1, 3> H_rcv) {
  const Point3 dr = sat - rcv;
  const double r = dr.norm();
  const double sagnac =
      OMGE * (sat.x() * rcv.y() - sat.y() * rcv.x()) / C_LIGHT;
  if (r < std::numeric_limits<double>::epsilon()) {
    e = Point3::Zero();
    if (H_rcv) H_rcv->setZero();
    return sagnac;
  }
  e = dr / r;
  if (H_rcv) {
    // d(||sat - rcv||)/d(rcv) = -e^T (1x3)
    // d(sagnac)/d(rcv)        = OMGE / C_LIGHT * [-sat.y(), sat.x(), 0]
    (*H_rcv)(0, 0) = -e.x() - OMGE * sat.y() / C_LIGHT;
    (*H_rcv)(0, 1) = -e.y() + OMGE * sat.x() / C_LIGHT;
    (*H_rcv)(0, 2) = -e.z();
  }
  return r + sagnac;
}

//*****************************************************************************
double DoubleDifferenceData::model(const Point3& rcv,
                                   OptionalJacobian<1, 3> H_rcv) const {
  Point3 e;
  Matrix13 H_rovRef, H_rovTarget;
  const double rRovRef =
      geodist(satRefRov, rcv, e, H_rcv ? &H_rovRef : nullptr);
  const double rRovTarget =
      geodist(satTargetRov, rcv, e, H_rcv ? &H_rovTarget : nullptr);
  const double rBaseRef = geodist(satRefBase, basePos, e);
  const double rBaseTarget = geodist(satTargetBase, basePos, e);
  if (H_rcv) *H_rcv = H_rovRef - H_rovTarget;
  return (rRovRef - rBaseRef) - (rRovTarget - rBaseTarget);
}

//*****************************************************************************
bool DoubleDifferenceData::equals(const DoubleDifferenceData& other,
                                  double tol) const {
  return std::abs(rovRef - other.rovRef) < tol &&
         std::abs(baseRef - other.baseRef) < tol &&
         std::abs(rovTarget - other.rovTarget) < tol &&
         std::abs(baseTarget - other.baseTarget) < tol &&
         traits<Point3>::Equals(satRefRov, other.satRefRov, tol) &&
         traits<Point3>::Equals(satTargetRov, other.satTargetRov, tol) &&
         traits<Point3>::Equals(satRefBase, other.satRefBase, tol) &&
         traits<Point3>::Equals(satTargetBase, other.satTargetBase, tol) &&
         traits<Point3>::Equals(basePos, other.basePos, tol);
}

//*****************************************************************************
Point3 LeverArm::antennaPosition(const Pose3& pose, PoseFrame* frame) const {
  const bool has_nav = ecef_T_nav.has_value();
  const Pose3 ecef_T_body = has_nav
      ? ecef_T_nav->compose(pose, {}, frame ? &frame->H_compose : nullptr)
      : pose;
  const Matrix3 R = ecef_T_body.rotation().matrix();
  if (frame) {
    frame->ecef_R_body = R;
    frame->has_nav = has_nav;
  }
  return ecef_T_body.translation() + R * b;
}

//*****************************************************************************
Matrix16 LeverArm::antennaPoseJacobian(const Matrix13& H_antenna,
                                       const PoseFrame& frame) const {
  // d(antenna)/d(pose tangent) = [-R*[b]_x | R], where R = ecef_R_body.
  Matrix16 H_ecef;
  H_ecef.block<1, 3>(0, 0) = H_antenna * (-frame.ecef_R_body * skewSymmetric(b));
  H_ecef.block<1, 3>(0, 3) = H_antenna * frame.ecef_R_body;
  return frame.has_nav ? Matrix16(H_ecef * frame.H_compose) : H_ecef;
}

//*****************************************************************************
bool LeverArm::equals(const LeverArm& other, double tol) const {
  if (!traits<Point3>::Equals(b, other.b, tol)) return false;
  if (ecef_T_nav.has_value() != other.ecef_T_nav.has_value()) return false;
  if (ecef_T_nav && !ecef_T_nav->equals(*other.ecef_T_nav, tol)) return false;
  return true;
}

}  // namespace gnss
}  // namespace gtsam
