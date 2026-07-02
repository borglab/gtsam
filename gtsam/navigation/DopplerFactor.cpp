/**
 *  @file   DopplerFactor.cpp
 *  @brief  Implementation of the GNSS Doppler factor and constant-drift clock
 *  @date   June 17, 2026
 **/

#include "DopplerFactor.h"

namespace gtsam {

using gnss::C_LIGHT;
using gnss::OMGE;

//***************************************************************************
DopplerFactor::DopplerFactor(const Key velocityKey, const Key clockDriftKey,
                             const double measuredDoppler,
                             const double wavelength,
                             const Point3& satellitePosition,
                             const Point3& satelliteVelocity,
                             const Point3& receiverPosition,
                             const double satelliteClockDrift,
                             const SharedNoiseModel& model)
    : Base(model, velocityKey, clockDriftKey),
      measRangeRate_(-wavelength * measuredDoppler),
      satVel_(satelliteVelocity),
      satClkDrift_(satelliteClockDrift) {
  // Line-of-sight unit vector (receiver -> satellite), Sagnac-aware geodist.
  Point3 e;
  gnss::geodist(satellitePosition, receiverPosition, e);
  los_ = e;

  // Earth-rotation (Sagnac) rate term, matching RTKLIB resdop():
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
  gtsam::print(Vector(velSagnac_), "Sagnac rate coeff (1/s): ");
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
         traits<Point3>::Equals(velSagnac_, e->velSagnac_, tol) &&
         traits<double>::Equals(sagnacOffset_, e->sagnacOffset_, tol);
}

//***************************************************************************
Vector DopplerFactor::evaluateError(const Vector3& velocity,
                                    const double& clockDrift,
                                    OptionalMatrixType Hvelocity,
                                    OptionalMatrixType HclockDrift) const {
  // range rate = e . (v_s - v_r) + c * (ddt_r - ddt_s) + sagnac_rate
  const double rangeRate = los_.dot(satVel_ - velocity) +
                           C_LIGHT * (clockDrift - satClkDrift_) +
                           sagnacOffset_ + velSagnac_.dot(velocity);
  const double error = rangeRate - measRangeRate_;

  if (Hvelocity) {
    // d/d v_r [ e . (v_s - v_r) + velSagnac . v_r ] = (velSagnac - e)^T
    *Hvelocity = (velSagnac_ - los_).transpose();
  }
  if (HclockDrift) {
    *HclockDrift = I_1x1 * C_LIGHT;
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
    const Key poseKey, const Key velocityKey, const Key clockDriftKey,
    const double measuredDoppler, const double wavelength,
    const Point3& satellitePosition, const Point3& satelliteVelocity,
    const Point3& receiverPosition, const Point3& leverArm,
    const Point3& angularVelocity, const double satelliteClockDrift,
    const SharedNoiseModel& model)
    : Base(model, poseKey, velocityKey, clockDriftKey),
      measRangeRate_(-wavelength * measuredDoppler),
      satVel_(satelliteVelocity),
      satClkDrift_(satelliteClockDrift),
      arm_(leverArm) {
  initDopplerArmGeometry(satellitePosition, receiverPosition, satelliteVelocity,
                         angularVelocity, leverArm, los_, velSagnac_,
                         sagnacOffset_, leverVel_);
}

DopplerFactorArm::DopplerFactorArm(
    const Key poseKey, const Key velocityKey, const Key clockDriftKey,
    const double measuredDoppler, const double wavelength,
    const Point3& satellitePosition, const Point3& satelliteVelocity,
    const Point3& receiverPosition, const Point3& leverArm,
    const Pose3& ecef_T_nav, const Point3& angularVelocity,
    const double satelliteClockDrift, const SharedNoiseModel& model)
    : Base(model, poseKey, velocityKey, clockDriftKey),
      measRangeRate_(-wavelength * measuredDoppler),
      satVel_(satelliteVelocity),
      satClkDrift_(satelliteClockDrift),
      arm_(leverArm, ecef_T_nav) {
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
  gtsam::print(sagnacOffset_, "Sagnac rate offset (m/s): ");
  gtsam::print(Vector(arm_.b), "lever arm (body m): ");
  gtsam::print(Vector(leverVel_), "lever velocity omega x b (m/s): ");
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
         traits<Point3>::Equals(velSagnac_, e->velSagnac_, tol) &&
         traits<double>::Equals(sagnacOffset_, e->sagnacOffset_, tol) &&
         arm_.equals(e->arm_, tol) &&
         traits<Point3>::Equals(leverVel_, e->leverVel_, tol);
}

//***************************************************************************
Vector DopplerFactorArm::evaluateError(const Pose3& pose,
                                       const Vector3& velocity,
                                       const double& clockDrift,
                                       OptionalMatrixType Hpose,
                                       OptionalMatrixType Hvelocity,
                                       OptionalMatrixType HclockDrift) const {
  // Lever-arm velocity in ECEF: v_lever = ecef_R_body * (omega x leverArm).
  // Hrot is d(v_lever)/d(rotation tangent) [3x3].
  Matrix3 Hrot;
  Point3 leverVelEcef;
  if (arm_.ecef_T_nav) {
    Matrix3 Hinner;
    const Point3 vNav = pose.rotation().rotate(leverVel_, Hinner);
    const Matrix3 Recn = arm_.ecef_T_nav->rotation().matrix();
    leverVelEcef = arm_.ecef_T_nav->rotation().rotate(vNav);
    Hrot = Recn * Hinner;
  } else {
    leverVelEcef = pose.rotation().rotate(leverVel_, Hrot);
  }
  const Vector3 vAnt = velocity + Vector3(leverVelEcef);

  // Effective range-rate coefficient on the antenna velocity: (velSagnac - e).
  const Vector3 g = Vector3(velSagnac_) - Vector3(los_);
  const double rangeRate = los_.dot(satVel_) +
                           C_LIGHT * (clockDrift - satClkDrift_) +
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
    *Hvelocity = g.transpose();
  }
  if (HclockDrift) {
    *HclockDrift = I_1x1 * C_LIGHT;
  }

  return Vector1(error);
}

//***************************************************************************
ClockDriftFactor::ClockDriftFactor(const Key clockBiasPrevKey,
                                   const Key clockBiasCurrKey,
                                   const Key clockDriftKey, const double dt,
                                   const SharedNoiseModel& model)
    : Base(model, clockBiasPrevKey, clockBiasCurrKey, clockDriftKey), dt_(dt) {}

//***************************************************************************
void ClockDriftFactor::print(const std::string& s,
                             const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(dt_, "dt (s): ");
}

//***************************************************************************
bool ClockDriftFactor::equals(const NonlinearFactor& expected,
                              double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(dt_, e->dt_, tol);
}

//***************************************************************************
Vector ClockDriftFactor::evaluateError(const double& clockBiasPrev,
                                       const double& clockBiasCurr,
                                       const double& clockDrift,
                                       OptionalMatrixType HbiasPrev,
                                       OptionalMatrixType HbiasCurr,
                                       OptionalMatrixType Hdrift) const {
  // Constant-velocity clock: bias(k) - bias(k-1) - drift*dt = 0
  const double error = clockBiasCurr - clockBiasPrev - clockDrift * dt_;

  if (HbiasPrev) *HbiasPrev = -I_1x1;
  if (HbiasCurr) *HbiasCurr = I_1x1;
  if (Hdrift) *Hdrift = -I_1x1 * dt_;

  return Vector1(error);
}

}  // namespace gtsam
