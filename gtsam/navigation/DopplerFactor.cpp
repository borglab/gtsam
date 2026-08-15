/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   DopplerFactor.cpp
 *  @brief  Implementation of the GNSS Doppler (range-rate) factor
 *  @date   July 2026
 **/

#include "DopplerFactor.h"

#include <stdexcept>

namespace gtsam {

using gnss::C_LIGHT;
using gnss::OMGE;

//***************************************************************************
DopplerFactor::DopplerFactor(const Key velocityKey, const Key clockBiasPrevKey,
                             const Key clockBiasCurrKey,
                             const double measuredDoppler,
                             const double wavelength,
                             const Point3& satellitePosition,
                             const Point3& satelliteVelocity,
                             const Point3& receiverPosition, const double dt,
                             const double satelliteClockDrift,
                             const SharedNoiseModel& model)
    : Base(model, velocityKey, clockBiasPrevKey, clockBiasCurrKey),
      measRangeRate_(-wavelength * measuredDoppler),
      satVel_(satelliteVelocity),
      satClkDrift_(satelliteClockDrift),
      dt_(dt) {
  if (!(dt > 0.0))
    throw std::invalid_argument("DopplerFactor: dt must be positive");

  // Line-of-sight unit vector (receiver -> satellite), Sagnac-aware geodist.
  Point3 e;
  gnss::geodist(satellitePosition, receiverPosition, e);
  los_ = e;

  // Earth-rotation (Sagnac) rate term:
  //   (OMGE/c) * (v_s.y*r_r.x + r_s.y*v_r.x - v_s.x*r_r.y - r_s.x*v_r.y)
  // Split into the v_r-independent offset and the linear coefficient on v_r.
  const double k = OMGE / C_LIGHT;
  sagnacOffset_ = k * (satelliteVelocity.y() * receiverPosition.x() -
                       satelliteVelocity.x() * receiverPosition.y());
  velSagnac_ = Point3(k * satellitePosition.y(), -k * satellitePosition.x(), 0.0);
}

//***************************************************************************
void DopplerFactor::print(const std::string& s,
                          const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measRangeRate_, "measured range rate (m/s): ");
  gtsam::print(Vector(satVel_), "sat velocity (ECEF m/s): ");
  gtsam::print(Vector(los_), "line-of-sight (rcv->sat): ");
  gtsam::print(satClkDrift_, "sat clock drift (s/s): ");
  gtsam::print(dt_, "epoch interval dt (s): ");
  gtsam::print(Vector(velSagnac_), "Sagnac rate coeff (unitless): ");
  gtsam::print(sagnacOffset_, "Sagnac rate offset (m/s): ");
}

//***************************************************************************
bool DopplerFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measRangeRate_, e->measRangeRate_, tol) &&
         traits<Point3>::Equals(satVel_, e->satVel_, tol) &&
         traits<Point3>::Equals(los_, e->los_, tol) &&
         traits<double>::Equals(satClkDrift_, e->satClkDrift_, tol) &&
         traits<double>::Equals(dt_, e->dt_, tol) &&
         traits<Point3>::Equals(velSagnac_, e->velSagnac_, tol) &&
         traits<double>::Equals(sagnacOffset_, e->sagnacOffset_, tol);
}

//***************************************************************************
Vector1 DopplerFactor::evaluateError(const Vector3& velocity,
                                     const double& clockBiasPrev,
                                     const double& clockBiasCurr,
                                     OptionalMatrixType Hvelocity,
                                     OptionalMatrixType HclockBiasPrev,
                                     OptionalMatrixType HclockBiasCurr) const {
  // range rate = e . (v_s - v_r)
  //            + c * ((bias_k - bias_{k-1})/dt - ddt_s) + sagnac_rate
  const double drift = (clockBiasCurr - clockBiasPrev) / dt_;
  const double rangeRate = los_.dot(satVel_ - velocity) +
                           C_LIGHT * (drift - satClkDrift_) +
                           sagnacOffset_ + velSagnac_.dot(velocity);
  const double error = rangeRate - measRangeRate_;

  if (Hvelocity) {
    // d/d v_r [ e . (v_s - v_r) + velSagnac . v_r ] = (velSagnac - e)^T
    *Hvelocity = (velSagnac_ - los_).transpose();
  }
  if (HclockBiasPrev) {
    *HclockBiasPrev = -I_1x1 * (C_LIGHT / dt_);
  }
  if (HclockBiasCurr) {
    *HclockBiasCurr = I_1x1 * (C_LIGHT / dt_);
  }

  return Vector1(error);
}

//***************************************************************************
// DopplerFactorArm
//***************************************************************************
namespace {
// Shared geometry precompute for the two DopplerFactorArm constructors: the
// LOS unit vector and the (nominal-position) Sagnac rate terms, identical to
// DopplerFactor, plus the body-frame lever-arm velocity omega x b.
void initDopplerArmGeometry(const Point3& satellitePosition,
                            const Point3& receiverPosition,
                            const Point3& satelliteVelocity,
                            const Point3& angularVelocity,
                            const Point3& leverArm, Point3& los, Point3& velSagnac,
                            double& sagnacOffset, Point3& leverVel) {
  gnss::geodist(satellitePosition, receiverPosition, los);
  const double k = OMGE / C_LIGHT;
  sagnacOffset = k * (satelliteVelocity.y() * receiverPosition.x() -
                      satelliteVelocity.x() * receiverPosition.y());
  velSagnac = Point3(k * satellitePosition.y(), -k * satellitePosition.x(), 0.0);
  leverVel = angularVelocity.cross(leverArm);
}
}  // namespace

DopplerFactorArm::DopplerFactorArm(
    const Key poseKey, const Key velocityKey, const Key clockBiasPrevKey,
    const Key clockBiasCurrKey, const double measuredDoppler,
    const double wavelength, const Point3& satellitePosition,
    const Point3& satelliteVelocity, const Point3& receiverPosition,
    const Point3& leverArm, const Point3& angularVelocity, const double dt,
    const double satelliteClockDrift, const SharedNoiseModel& model)
    : Base(model, poseKey, velocityKey, clockBiasPrevKey, clockBiasCurrKey),
      measRangeRate_(-wavelength * measuredDoppler),
      satVel_(satelliteVelocity),
      satClkDrift_(satelliteClockDrift),
      dt_(dt),
      arm_(leverArm) {
  if (!(dt > 0.0))
    throw std::invalid_argument("DopplerFactorArm: dt must be positive");
  initDopplerArmGeometry(satellitePosition, receiverPosition, satelliteVelocity,
                         angularVelocity, leverArm, los_, velSagnac_,
                         sagnacOffset_, leverVel_);
}

DopplerFactorArm::DopplerFactorArm(
    const Key poseKey, const Key velocityKey, const Key clockBiasPrevKey,
    const Key clockBiasCurrKey, const double measuredDoppler,
    const double wavelength, const Point3& satellitePosition,
    const Point3& satelliteVelocity, const Point3& receiverPosition,
    const Point3& leverArm, const Pose3& ecef_T_nav,
    const Point3& angularVelocity, const double dt,
    const double satelliteClockDrift, const SharedNoiseModel& model)
    : Base(model, poseKey, velocityKey, clockBiasPrevKey, clockBiasCurrKey),
      measRangeRate_(-wavelength * measuredDoppler),
      satVel_(satelliteVelocity),
      satClkDrift_(satelliteClockDrift),
      dt_(dt),
      arm_(leverArm, ecef_T_nav) {
  if (!(dt > 0.0))
    throw std::invalid_argument("DopplerFactorArm: dt must be positive");
  initDopplerArmGeometry(satellitePosition, receiverPosition, satelliteVelocity,
                         angularVelocity, leverArm, los_, velSagnac_,
                         sagnacOffset_, leverVel_);
}

//***************************************************************************
void DopplerFactorArm::print(const std::string& s,
                             const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measRangeRate_, "measured range rate (m/s): ");
  gtsam::print(Vector(satVel_), "sat velocity (ECEF m/s): ");
  gtsam::print(Vector(los_), "line-of-sight (rcv->sat): ");
  gtsam::print(satClkDrift_, "sat clock drift (s/s): ");
  gtsam::print(dt_, "epoch interval dt (s): ");
  gtsam::print(Vector(velSagnac_), "Sagnac rate coeff (unitless): ");
  gtsam::print(sagnacOffset_, "Sagnac rate offset (m/s): ");
  gtsam::print(Vector(arm_.b), "lever arm (body m): ");
  gtsam::print(Vector(leverVel_), "lever velocity omega x b (m/s): ");
  if (arm_.ecef_T_nav) arm_.ecef_T_nav->print("ecef_T_nav: ");
}

//***************************************************************************
bool DopplerFactorArm::equals(const NonlinearFactor& expected,
                              double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measRangeRate_, e->measRangeRate_, tol) &&
         traits<Point3>::Equals(satVel_, e->satVel_, tol) &&
         traits<Point3>::Equals(los_, e->los_, tol) &&
         traits<double>::Equals(satClkDrift_, e->satClkDrift_, tol) &&
         traits<double>::Equals(dt_, e->dt_, tol) &&
         traits<Point3>::Equals(velSagnac_, e->velSagnac_, tol) &&
         traits<double>::Equals(sagnacOffset_, e->sagnacOffset_, tol) &&
         arm_.equals(e->arm_, tol) &&
         traits<Point3>::Equals(leverVel_, e->leverVel_, tol);
}

//***************************************************************************
Vector DopplerFactorArm::evaluateError(
    const Pose3& pose, const Vector3& velocity, const double& clockBiasPrev,
    const double& clockBiasCurr, OptionalMatrixType Hpose,
    OptionalMatrixType Hvelocity, OptionalMatrixType HclockBiasPrev,
    OptionalMatrixType HclockBiasCurr) const {
  // Antenna velocity in ECEF: v_ant = Rvel*velocity + ecef_R_body*(omega x b).
  // With ecef_T_nav the pose and velocity are nav-frame, so Rvel = ecef_R_nav
  // rotates them to ECEF; otherwise both are already ECEF (Rvel = I).
  Matrix3 Hrot;
  Matrix3 Rvel = I_3x3;
  Point3 leverVelEcef;
  Vector3 velEcef;
  if (arm_.ecef_T_nav) {
    Matrix3 Hinner;
    const Point3 vNav = pose.rotation().rotate(leverVel_, Hinner);
    Rvel = arm_.ecef_T_nav->rotation().matrix();
    leverVelEcef = arm_.ecef_T_nav->rotation().rotate(vNav);
    Hrot = Rvel * Hinner;
    velEcef = Rvel * velocity;
  } else {
    leverVelEcef = pose.rotation().rotate(leverVel_, Hrot);
    velEcef = velocity;
  }
  const Vector3 vAnt = velEcef + Vector3(leverVelEcef);

  // Effective range-rate coefficient on the antenna velocity: (velSagnac - e).
  const Vector3 g = Vector3(velSagnac_) - Vector3(los_);
  const double drift = (clockBiasCurr - clockBiasPrev) / dt_;
  const double rangeRate = los_.dot(satVel_) +
                           C_LIGHT * (drift - satClkDrift_) +
                           sagnacOffset_ + g.dot(vAnt);
  const double error = rangeRate - measRangeRate_;

  if (Hpose) {
    // Pose enters only through v_ant = ... + R*(omega x b); the LOS/Sagnac use
    // the fixed nominal position, so the translation block is zero. Pose3
    // tangent order is [rotation(3), translation(3)].
    Matrix16 H = Matrix16::Zero();
    H.block<1, 3>(0, 0) = g.transpose() * Hrot;
    *Hpose = H;
  }
  if (Hvelocity) {
    // d(error)/d(velocity) = g^T * d(v_ant)/d(velocity) = g^T * R_vel.
    *Hvelocity = g.transpose() * Rvel;
  }
  if (HclockBiasPrev) {
    *HclockBiasPrev = -I_1x1 * (C_LIGHT / dt_);
  }
  if (HclockBiasCurr) {
    *HclockBiasCurr = I_1x1 * (C_LIGHT / dt_);
  }

  return Vector1(error);
}

}  // namespace gtsam
