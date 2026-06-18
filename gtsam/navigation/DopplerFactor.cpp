/**
 *  @file   DopplerFactor.cpp
 *  @brief  Implementation of the GNSS Doppler factor and constant-drift clock
 *  @date   June 17, 2026
 **/

#include "DopplerFactor.h"

namespace gtsam {

using gnss::C_LIGHT;

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
}

//***************************************************************************
void DopplerFactor::print(const std::string& s,
                          const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measRangeRate_, "measured range rate (m/s): ");
  gtsam::print(Vector(satVel_), "sat velocity (ECEF m/s): ");
  gtsam::print(Vector(los_), "line-of-sight (rcv->sat): ");
  gtsam::print(satClkDrift_, "sat clock drift (s/s): ");
}

//***************************************************************************
bool DopplerFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measRangeRate_, e->measRangeRate_, tol) &&
         traits<Point3>::Equals(satVel_, e->satVel_, tol) &&
         traits<Point3>::Equals(los_, e->los_, tol) &&
         traits<double>::Equals(satClkDrift_, e->satClkDrift_, tol);
}

//***************************************************************************
Vector DopplerFactor::evaluateError(const Vector3& velocity,
                                    const double& clockDrift,
                                    OptionalMatrixType Hvelocity,
                                    OptionalMatrixType HclockDrift) const {
  // range rate = e . (v_s - v_r) + c * (ddt_r - ddt_s)
  const double rangeRate =
      los_.dot(satVel_ - velocity) + C_LIGHT * (clockDrift - satClkDrift_);
  const double error = rangeRate - measRangeRate_;

  if (Hvelocity) {
    *Hvelocity = -los_.transpose();  // d/d v_r [ e . (v_s - v_r) ] = -e^T
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
