/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   CarrierPhaseFactor.cpp
 *  @brief  Implementation file for GNSS Carrier Phase factors
 *  @date   March 23, 2026
 **/

#include "CarrierPhaseFactor.h"

#include <gtsam/base/MatrixConstants.h>

namespace gtsam {

using gnss::C_LIGHT;

//***************************************************************************
CarrierPhaseFactor::CarrierPhaseFactor(
    const Key receiverPositionKey, const Key receiverClockBiasKey,
    const Key ambiguityKey, const double measuredCarrierPhaseMeters,
    const Point3& satellitePosition, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey, ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhaseMeters, satellitePosition,
                       satelliteClockBias} {}

//***************************************************************************
void CarrierPhaseFactor::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "carrier phase (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool CarrierPhaseFactor::equals(const NonlinearFactor& expected,
                                double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector1 CarrierPhaseFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClockBias,
    const double& ambiguity, OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType Hambiguity) const {
  // error = range + c*(dt_u - dt_s) + ambiguity - measurement
  // Sagnac-corrected geodist keeps the geometry consistent with the DD factors.
  Point3 e;
  Matrix13 H_geo;
  const double range =
      gnss::geodist(satPos_, receiverPosition, e, HreceiverPos ? &H_geo : nullptr);
  const double rho =
      range + C_LIGHT * (receiverClockBias - satClkBias_) + ambiguity;
  const double error = rho - measurement_;

  if (HreceiverPos) {
    *HreceiverPos = H_geo;
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * C_LIGHT;
  }

  if (Hambiguity) {
    *Hambiguity = I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
UndifferencedCarrierPhaseFactor::UndifferencedCarrierPhaseFactor(
    const Key receiverPositionKey, const Key receiverClockBiasKey,
    const Key tropoZenithWetKey, const Key slantIonoKey, const Key ambiguityKey,
    const double measuredCarrierPhaseMeters, const Point3& satellitePosition,
    const double tropoWetMapping, const double ionoCoefficient,
    const double lambda, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey, tropoZenithWetKey,
           slantIonoKey, ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhaseMeters, satellitePosition,
                       satelliteClockBias},
      tropoMap_(tropoWetMapping),
      ionoCoeff_(ionoCoefficient),
      lambda_(lambda) {}

//***************************************************************************
void UndifferencedCarrierPhaseFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "carrier phase (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(tropoMap_, "tropo wet mapping: ");
  gtsam::print(ionoCoeff_, "iono coefficient: ");
  gtsam::print(lambda_, "wavelength (m): ");
}

//***************************************************************************
bool UndifferencedCarrierPhaseFactor::equals(const NonlinearFactor& expected,
                                          double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol) &&
         traits<double>::Equals(tropoMap_, e->tropoMap_, tol) &&
         traits<double>::Equals(ionoCoeff_, e->ionoCoeff_, tol) &&
         traits<double>::Equals(lambda_, e->lambda_, tol);
}

//***************************************************************************
Vector UndifferencedCarrierPhaseFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClockBias,
    const double& tropoZenithWet, const double& slantIono,
    const double& ambiguity, OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias, OptionalMatrixType HtropoZenithWet,
    OptionalMatrixType HslantIono, OptionalMatrixType Hambiguity) const {
  // Undifferenced PPP carrier phase model (iono advanced, +lambda*N):
  //   rho = geodist + c*(dt_u - dt_s) + m_w*ZTD - mu_f*I_slant + lambda*N
  Point3 e;
  Matrix13 H_geo;
  const double range = gnss::geodist(satPos_, receiverPosition, e,
                                     HreceiverPos ? &H_geo : nullptr);
  const double rho = range + C_LIGHT * (receiverClockBias - satClkBias_) +
                     tropoMap_ * tropoZenithWet - ionoCoeff_ * slantIono +
                     lambda_ * ambiguity;
  const double error = rho - measurement_;

  if (HreceiverPos) *HreceiverPos = H_geo;
  if (HreceiverClockBias) *HreceiverClockBias = I_1x1 * C_LIGHT;
  if (HtropoZenithWet) *HtropoZenithWet = I_1x1 * tropoMap_;
  if (HslantIono) *HslantIono = I_1x1 * (-ionoCoeff_);
  if (Hambiguity) *Hambiguity = I_1x1 * lambda_;

  return Vector1(error);
}

//***************************************************************************
UndifferencedCarrierPhaseFactorArm::UndifferencedCarrierPhaseFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const Key tropoZenithWetKey, const Key slantIonoKey, const Key ambiguityKey,
    const double measuredCarrierPhaseMeters, const Point3& satellitePosition,
    const Point3& leverArm, const double tropoWetMapping,
    const double ionoCoefficient, const double lambda,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, tropoZenithWetKey, slantIonoKey,
           ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhaseMeters, satellitePosition,
                       satelliteClockBias},
      arm_(leverArm),
      tropoMap_(tropoWetMapping),
      ionoCoeff_(ionoCoefficient),
      lambda_(lambda) {}

//***************************************************************************
UndifferencedCarrierPhaseFactorArm::UndifferencedCarrierPhaseFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const Key tropoZenithWetKey, const Key slantIonoKey, const Key ambiguityKey,
    const double measuredCarrierPhaseMeters, const Point3& satellitePosition,
    const Point3& leverArm, const Pose3& ecef_T_nav,
    const double tropoWetMapping, const double ionoCoefficient,
    const double lambda, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, tropoZenithWetKey, slantIonoKey,
           ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhaseMeters, satellitePosition,
                       satelliteClockBias},
      arm_(leverArm, ecef_T_nav),
      tropoMap_(tropoWetMapping),
      ionoCoeff_(ionoCoefficient),
      lambda_(lambda) {}

//***************************************************************************
void UndifferencedCarrierPhaseFactorArm::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "carrier phase (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(Vector(arm_.b), "lever arm (body frame meters): ");
  gtsam::print(tropoMap_, "tropo wet mapping: ");
  gtsam::print(ionoCoeff_, "iono coefficient: ");
  gtsam::print(lambda_, "wavelength (m): ");
  if (arm_.ecef_T_nav) {
    arm_.ecef_T_nav->print("ecef_T_nav:\n");
  }
}

//***************************************************************************
bool UndifferencedCarrierPhaseFactorArm::equals(const NonlinearFactor& expected,
                                             double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol) &&
         arm_.equals(e->arm_, tol) &&
         traits<double>::Equals(tropoMap_, e->tropoMap_, tol) &&
         traits<double>::Equals(ionoCoeff_, e->ionoCoeff_, tol) &&
         traits<double>::Equals(lambda_, e->lambda_, tol);
}

//***************************************************************************
Vector UndifferencedCarrierPhaseFactorArm::evaluateError(
    const Pose3& pose, const double& receiverClockBias,
    const double& tropoZenithWet, const double& slantIono,
    const double& ambiguity, OptionalMatrixType H_pose,
    OptionalMatrixType HreceiverClockBias, OptionalMatrixType HtropoZenithWet,
    OptionalMatrixType HslantIono, OptionalMatrixType Hambiguity) const {
  gnss::LeverArm::PoseFrame frame;
  const Point3 antennaPos =
      arm_.antennaPosition(pose, H_pose ? &frame : nullptr);

  Point3 e;
  Matrix13 H_antenna;
  const double range =
      gnss::geodist(satPos_, antennaPos, e, H_pose ? &H_antenna : nullptr);
  const double rho = range + C_LIGHT * (receiverClockBias - satClkBias_) +
                     tropoMap_ * tropoZenithWet - ionoCoeff_ * slantIono +
                     lambda_ * ambiguity;
  const double error = rho - measurement_;

  if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_antenna, frame);
  if (HreceiverClockBias) *HreceiverClockBias = I_1x1 * C_LIGHT;
  if (HtropoZenithWet) *HtropoZenithWet = I_1x1 * tropoMap_;
  if (HslantIono) *HslantIono = I_1x1 * (-ionoCoeff_);
  if (Hambiguity) *Hambiguity = I_1x1 * lambda_;

  return Vector1(error);
}

//***************************************************************************
CarrierPhaseFactorArm::CarrierPhaseFactorArm(
    const Key poseKey, const Key receiverClockBiasKey, const Key ambiguityKey,
    const double measuredCarrierPhaseMeters, const Point3& satellitePosition,
    const Point3& leverArm, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhaseMeters, satellitePosition,
                       satelliteClockBias},
      arm_(leverArm) {}

//***************************************************************************
CarrierPhaseFactorArm::CarrierPhaseFactorArm(
    const Key poseKey, const Key receiverClockBiasKey, const Key ambiguityKey,
    const double measuredCarrierPhaseMeters, const Point3& satellitePosition,
    const Point3& leverArm, const Pose3& ecef_T_nav,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhaseMeters, satellitePosition,
                       satelliteClockBias},
      arm_(leverArm, ecef_T_nav) {}

//***************************************************************************
void CarrierPhaseFactorArm::print(const std::string& s,
                                  const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "carrier phase (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(Vector(arm_.b), "lever arm (body frame meters): ");
  if (arm_.ecef_T_nav) {
    arm_.ecef_T_nav->print("ecef_T_nav:\n");
  }
}

//***************************************************************************
bool CarrierPhaseFactorArm::equals(const NonlinearFactor& expected,
                                   double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol) &&
         arm_.equals(e->arm_, tol);
}

//***************************************************************************
Vector1 CarrierPhaseFactorArm::evaluateError(
    const Pose3& pose, const double& receiverClockBias,
    const double& ambiguity, OptionalMatrixType H_pose,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType Hambiguity) const {
  gnss::LeverArm::PoseFrame frame;
  const Point3 antennaPos =
      arm_.antennaPosition(pose, H_pose ? &frame : nullptr);

  // error = range + c*(dt_u - dt_s) + ambiguity - measurement
  // Sagnac-corrected geodist keeps the geometry consistent with the DD factors.
  Point3 e;
  Matrix13 H_antenna;
  const double range =
      gnss::geodist(satPos_, antennaPos, e, H_pose ? &H_antenna : nullptr);
  const double rho =
      range + C_LIGHT * (receiverClockBias - satClkBias_) + ambiguity;
  const double error = rho - measurement_;

  if (H_pose) {
    *H_pose = arm_.antennaPoseJacobian(H_antenna, frame);
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * C_LIGHT;
  }

  if (Hambiguity) {
    *Hambiguity = I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
DoubleDifferenceCarrierPhaseFactor::DoubleDifferenceCarrierPhaseFactor(
    const Key positionKey, const Key ambRefKey, const Key ambTargetKey,
    const double cpRovRefMeters, const double cpBaseRefMeters,
    const double cpRovTargetMeters, const double cpBaseTargetMeters,
    const Point3& satRefRov, const Point3& satTargetRov,
    const Point3& satRefBase, const Point3& satTargetBase,
    const Point3& basePos, const double lam,
    const SharedNoiseModel& model)
    : Base(model, positionKey, ambRefKey, ambTargetKey),
      dd_{cpRovRefMeters, cpBaseRefMeters, cpRovTargetMeters,
          cpBaseTargetMeters, satRefRov, satTargetRov, satRefBase,
          satTargetBase, basePos},
      lam_(lam) {}

//***************************************************************************
void DoubleDifferenceCarrierPhaseFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ")
            << "DoubleDifferenceCarrierPhaseFactor\n";
  std::cout << "  lam: " << lam_ << "\n";
  Base::print("", keyFormatter);
}

//***************************************************************************
bool DoubleDifferenceCarrierPhaseFactor::equals(
    const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && dd_.equals(e->dd_, tol) &&
         std::abs(lam_ - e->lam_) < tol;
}

//***************************************************************************
Vector1 DoubleDifferenceCarrierPhaseFactor::evaluateError(
    const Point3& pos, const double& ambRef, const double& ambTarget,
    OptionalMatrixType Hpos, OptionalMatrixType HambRef,
    OptionalMatrixType HambTarget) const {
  Matrix13 H_pos;
  const double ddModel = dd_.model(pos, Hpos ? &H_pos : nullptr);
  const double error =
      ddModel + lam_ * (ambRef - ambTarget) - dd_.observed();

  if (Hpos) *Hpos = H_pos;
  if (HambRef) *HambRef = Matrix{{lam_}};
  if (HambTarget) *HambTarget = Matrix{{-lam_}};

  return Vector1(error);
}

//***************************************************************************
DoubleDifferenceCarrierPhaseFactorArm::DoubleDifferenceCarrierPhaseFactorArm(
    const Key poseKey, const Key ambRefKey, const Key ambTargetKey,
    const double cpRovRefMeters, const double cpBaseRefMeters,
    const double cpRovTargetMeters, const double cpBaseTargetMeters,
    const Point3& satRefRov, const Point3& satTargetRov,
    const Point3& satRefBase, const Point3& satTargetBase,
    const Point3& basePos, const double lam, const Point3& leverArm,
    const SharedNoiseModel& model)
    : Base(model, poseKey, ambRefKey, ambTargetKey),
      dd_{cpRovRefMeters, cpBaseRefMeters, cpRovTargetMeters,
          cpBaseTargetMeters, satRefRov, satTargetRov, satRefBase,
          satTargetBase, basePos},
      lam_(lam),
      arm_(leverArm) {}

//***************************************************************************
DoubleDifferenceCarrierPhaseFactorArm::DoubleDifferenceCarrierPhaseFactorArm(
    const Key poseKey, const Key ambRefKey, const Key ambTargetKey,
    const double cpRovRefMeters, const double cpBaseRefMeters,
    const double cpRovTargetMeters, const double cpBaseTargetMeters,
    const Point3& satRefRov, const Point3& satTargetRov,
    const Point3& satRefBase, const Point3& satTargetBase,
    const Point3& basePos, const double lam, const Point3& leverArm,
    const Pose3& ecef_T_nav, const SharedNoiseModel& model)
    : Base(model, poseKey, ambRefKey, ambTargetKey),
      dd_{cpRovRefMeters, cpBaseRefMeters, cpRovTargetMeters,
          cpBaseTargetMeters, satRefRov, satTargetRov, satRefBase,
          satTargetBase, basePos},
      lam_(lam),
      arm_(leverArm, ecef_T_nav) {}

//***************************************************************************
void DoubleDifferenceCarrierPhaseFactorArm::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ")
            << "DoubleDifferenceCarrierPhaseFactorArm\n";
  std::cout << "  lam: " << lam_ << "\n";
  Base::print("", keyFormatter);
}

//***************************************************************************
bool DoubleDifferenceCarrierPhaseFactorArm::equals(
    const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && dd_.equals(e->dd_, tol) &&
         std::abs(lam_ - e->lam_) < tol && arm_.equals(e->arm_, tol);
}

//***************************************************************************
Vector1 DoubleDifferenceCarrierPhaseFactorArm::evaluateError(
    const Pose3& pose, const double& ambRef, const double& ambTarget,
    OptionalMatrixType H_pose, OptionalMatrixType HambRef,
    OptionalMatrixType HambTarget) const {
  gnss::LeverArm::PoseFrame frame;
  const Point3 antennaPos =
      arm_.antennaPosition(pose, H_pose ? &frame : nullptr);

  Matrix13 H_antenna;
  const double ddModel =
      dd_.model(antennaPos, H_pose ? &H_antenna : nullptr);
  const double error =
      ddModel + lam_ * (ambRef - ambTarget) - dd_.observed();

  if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_antenna, frame);
  if (HambRef) *HambRef = Matrix{{lam_}};
  if (HambTarget) *HambTarget = Matrix{{-lam_}};

  return Vector1(error);
}

}  // namespace gtsam
