/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   PseudorangeFactor.cpp
 *  @author Sammy Guo
 *  @brief  Implementation file for GNSS Pseudorange factor
 *  @date   January 18, 2026
 **/

#include "PseudorangeFactor.h"

#include <gtsam/base/MatrixConstants.h>

namespace gtsam {

using gnss::C_LIGHT;

//***************************************************************************
PseudorangeFactor::PseudorangeFactor(const Key receiverPositionKey,
                                     const Key receiverClockBiasKey,
                                     const double measuredPseudorange,
                                     const Point3& satellitePosition,
                                     const double satelliteClockBias,
                                     const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias} {}

//***************************************************************************
void PseudorangeFactor::print(const std::string& s,
                              const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool PseudorangeFactor::equals(const NonlinearFactor& expected,
                               double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector1 PseudorangeFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClockBias,
    OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias) const {
  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  // Sagnac-corrected geodist keeps the geometry consistent with the DD factors.
  Point3 e;
  Matrix13 H_geo;
  const double range =
      gnss::geodist(satPos_, receiverPosition, e, HreceiverPos ? &H_geo : nullptr);
  const double rho = range + C_LIGHT * (receiverClockBias - satClkBias_);
  const double error = rho - measurement_;

  // Compute associated derivatives:
  if (HreceiverPos) {
    *HreceiverPos = H_geo;
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * C_LIGHT;
  }

  return Vector1(error);
}

//***************************************************************************
UndifferencedPseudorangeFactor::UndifferencedPseudorangeFactor(
    const Key receiverPositionKey, const Key receiverClockBiasKey,
    const Key tropoZenithWetKey, const Key slantIonoKey,
    const double measuredPseudorange, const Point3& satellitePosition,
    const double tropoWetMapping, const double ionoCoefficient,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey, tropoZenithWetKey,
           slantIonoKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      tropoMap_(tropoWetMapping),
      ionoCoeff_(ionoCoefficient) {}

//***************************************************************************
void UndifferencedPseudorangeFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(tropoMap_, "tropo wet mapping: ");
  gtsam::print(ionoCoeff_, "iono coefficient: ");
}

//***************************************************************************
bool UndifferencedPseudorangeFactor::equals(const NonlinearFactor& expected,
                                         double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol) &&
         traits<double>::Equals(tropoMap_, e->tropoMap_, tol) &&
         traits<double>::Equals(ionoCoeff_, e->ionoCoeff_, tol);
}

//***************************************************************************
Vector UndifferencedPseudorangeFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClockBias,
    const double& tropoZenithWet, const double& slantIono,
    OptionalMatrixType HreceiverPos, OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType HtropoZenithWet, OptionalMatrixType HslantIono) const {
  // Undifferenced PPP pseudorange model:
  //   rho = geodist(sat, rcv) + c*(dt_u - dt_s) + m_w*ZTD + mu_f*I_slant
  Point3 e;
  Matrix13 H_geo;
  const double range = gnss::geodist(satPos_, receiverPosition, e,
                                     HreceiverPos ? &H_geo : nullptr);
  const double rho = range + C_LIGHT * (receiverClockBias - satClkBias_) +
                     tropoMap_ * tropoZenithWet + ionoCoeff_ * slantIono;
  const double error = rho - measurement_;

  if (HreceiverPos) *HreceiverPos = H_geo;
  if (HreceiverClockBias) *HreceiverClockBias = I_1x1 * C_LIGHT;
  if (HtropoZenithWet) *HtropoZenithWet = I_1x1 * tropoMap_;
  if (HslantIono) *HslantIono = I_1x1 * ionoCoeff_;

  return Vector1(error);
}

//***************************************************************************
PseudorangeFactorArm::PseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const double measuredPseudorange, const Point3& satellitePosition,
    const Point3& leverArm, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      arm_(leverArm) {}

//***************************************************************************
PseudorangeFactorArm::PseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const double measuredPseudorange, const Point3& satellitePosition,
    const Point3& leverArm, const Pose3& ecef_T_nav,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      arm_(leverArm, ecef_T_nav) {}

//***************************************************************************
UndifferencedPseudorangeFactorArm::UndifferencedPseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const Key tropoZenithWetKey, const Key slantIonoKey,
    const double measuredPseudorange, const Point3& satellitePosition,
    const Point3& leverArm, const double tropoWetMapping,
    const double ionoCoefficient, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, tropoZenithWetKey,
           slantIonoKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      arm_(leverArm),
      tropoMap_(tropoWetMapping),
      ionoCoeff_(ionoCoefficient) {}

//***************************************************************************
UndifferencedPseudorangeFactorArm::UndifferencedPseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const Key tropoZenithWetKey, const Key slantIonoKey,
    const double measuredPseudorange, const Point3& satellitePosition,
    const Point3& leverArm, const Pose3& ecef_T_nav,
    const double tropoWetMapping, const double ionoCoefficient,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, tropoZenithWetKey,
           slantIonoKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      arm_(leverArm, ecef_T_nav),
      tropoMap_(tropoWetMapping),
      ionoCoeff_(ionoCoefficient) {}

//***************************************************************************
void UndifferencedPseudorangeFactorArm::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(Vector(arm_.b), "lever arm (body frame meters): ");
  gtsam::print(tropoMap_, "tropo wet mapping: ");
  gtsam::print(ionoCoeff_, "iono coefficient: ");
  if (arm_.ecef_T_nav) {
    arm_.ecef_T_nav->print("ecef_T_nav: ");
  }
}

//***************************************************************************
bool UndifferencedPseudorangeFactorArm::equals(const NonlinearFactor& expected,
                                            double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol) &&
         arm_.equals(e->arm_, tol) &&
         traits<double>::Equals(tropoMap_, e->tropoMap_, tol) &&
         traits<double>::Equals(ionoCoeff_, e->ionoCoeff_, tol);
}

//***************************************************************************
Vector UndifferencedPseudorangeFactorArm::evaluateError(
    const Pose3& pose, const double& receiverClockBias,
    const double& tropoZenithWet, const double& slantIono,
    OptionalMatrixType H_pose, OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType HtropoZenithWet, OptionalMatrixType HslantIono) const {
  gnss::LeverArm::PoseFrame frame;
  const Point3 antennaPos =
      arm_.antennaPosition(pose, H_pose ? &frame : nullptr);

  Point3 e;
  Matrix13 H_antenna;
  const double range =
      gnss::geodist(satPos_, antennaPos, e, H_pose ? &H_antenna : nullptr);
  const double rho = range + C_LIGHT * (receiverClockBias - satClkBias_) +
                     tropoMap_ * tropoZenithWet + ionoCoeff_ * slantIono;
  const double error = rho - measurement_;

  if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_antenna, frame);
  if (HreceiverClockBias) *HreceiverClockBias = I_1x1 * C_LIGHT;
  if (HtropoZenithWet) *HtropoZenithWet = I_1x1 * tropoMap_;
  if (HslantIono) *HslantIono = I_1x1 * ionoCoeff_;

  return Vector1(error);
}

//***************************************************************************
void PseudorangeFactorArm::print(const std::string& s,
                                  const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(Vector(arm_.b), "lever arm (body frame meters): ");
  if (arm_.ecef_T_nav) {
    arm_.ecef_T_nav->print("ecef_T_nav: ");
  }
}

//***************************************************************************
bool PseudorangeFactorArm::equals(const NonlinearFactor& expected,
                                   double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol) &&
         arm_.equals(e->arm_, tol);
}

//***************************************************************************
Vector1 PseudorangeFactorArm::evaluateError(
    const Pose3& pose, const double& receiverClockBias,
    OptionalMatrixType H_pose,
    OptionalMatrixType HreceiverClockBias) const {
  gnss::LeverArm::PoseFrame frame;
  const Point3 antennaPos =
      arm_.antennaPosition(pose, H_pose ? &frame : nullptr);

  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  // Sagnac-corrected geodist keeps the geometry consistent with the DD factors.
  Point3 e;
  Matrix13 H_antenna;
  const double range =
      gnss::geodist(satPos_, antennaPos, e, H_pose ? &H_antenna : nullptr);
  const double rho = range + C_LIGHT * (receiverClockBias - satClkBias_);
  const double error = rho - measurement_;

  if (H_pose) {
    *H_pose = arm_.antennaPoseJacobian(H_antenna, frame);
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * C_LIGHT;
  }

  return Vector1(error);
}

//***************************************************************************
DifferentialPseudorangeFactor::DifferentialPseudorangeFactor(
    const Key receiverPositionKey, const Key receiverClockBiasKey,
    const Key differentialCorrectionKey, const double measuredPseudorange,
    const Point3& satellitePosition, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey,
           differentialCorrectionKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias} {}

//***************************************************************************
void DifferentialPseudorangeFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool DifferentialPseudorangeFactor::equals(const NonlinearFactor& expected,
                                           double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector1 DifferentialPseudorangeFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClock_bias,
    const double& differentialCorrection, OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType HdifferentialCorrection) const {
  // Sagnac-corrected geodist keeps the geometry consistent with the DD factors.
  Point3 e;
  Matrix13 H_geo;
  const double range =
      gnss::geodist(satPos_, receiverPosition, e, HreceiverPos ? &H_geo : nullptr);
  const double rho = range + C_LIGHT * (receiverClock_bias - satClkBias_);
  const double error = rho - measurement_ - differentialCorrection;

  if (HreceiverPos) {
    *HreceiverPos = H_geo;
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * C_LIGHT;
  }

  if (HdifferentialCorrection) {
    *HdifferentialCorrection = -I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
DifferentialPseudorangeFactorArm::DifferentialPseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const Key differentialCorrectionKey, const double measuredPseudorange,
    const Point3& satellitePosition, const Point3& leverArm,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, differentialCorrectionKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      arm_(leverArm) {}

//***************************************************************************
DifferentialPseudorangeFactorArm::DifferentialPseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const Key differentialCorrectionKey, const double measuredPseudorange,
    const Point3& satellitePosition, const Point3& leverArm,
    const Pose3& ecef_T_nav, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, differentialCorrectionKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      arm_(leverArm, ecef_T_nav) {}

//***************************************************************************
void DifferentialPseudorangeFactorArm::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(measurement_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(Vector(arm_.b), "lever arm (body frame meters): ");
  if (arm_.ecef_T_nav) {
    arm_.ecef_T_nav->print("ecef_T_nav: ");
  }
}

//***************************************************************************
bool DifferentialPseudorangeFactorArm::equals(
    const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(measurement_, e->measurement_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol) &&
         arm_.equals(e->arm_, tol);
}

//***************************************************************************
Vector1 DifferentialPseudorangeFactorArm::evaluateError(
    const Pose3& pose, const double& receiverClockBias,
    const double& differentialCorrection, OptionalMatrixType H_pose,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType HdifferentialCorrection) const {
  gnss::LeverArm::PoseFrame frame;
  const Point3 antennaPos =
      arm_.antennaPosition(pose, H_pose ? &frame : nullptr);

  // Sagnac-corrected geodist keeps the geometry consistent with the DD factors.
  Point3 e;
  Matrix13 H_antenna;
  const double range =
      gnss::geodist(satPos_, antennaPos, e, H_pose ? &H_antenna : nullptr);
  const double rho = range + C_LIGHT * (receiverClockBias - satClkBias_);
  const double error = rho - measurement_ - differentialCorrection;

  if (H_pose) {
    *H_pose = arm_.antennaPoseJacobian(H_antenna, frame);
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * C_LIGHT;
  }

  if (HdifferentialCorrection) {
    *HdifferentialCorrection = -I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
DoubleDifferencePseudorangeFactor::DoubleDifferencePseudorangeFactor(
    const Key positionKey, const double prRovRef, const double prBaseRef,
    const double prRovTarget, const double prBaseTarget,
    const Point3& satRefRov, const Point3& satTargetRov,
    const Point3& satRefBase, const Point3& satTargetBase,
    const Point3& basePos, const SharedNoiseModel& model)
    : Base(model, positionKey),
      dd_{prRovRef, prBaseRef, prRovTarget, prBaseTarget,
          satRefRov, satTargetRov, satRefBase, satTargetBase, basePos} {}

//***************************************************************************
void DoubleDifferencePseudorangeFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ")
            << "DoubleDifferencePseudorangeFactor\n";
  Base::print("", keyFormatter);
}

//***************************************************************************
bool DoubleDifferencePseudorangeFactor::equals(const NonlinearFactor& expected,
                                               double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && dd_.equals(e->dd_, tol);
}

//***************************************************************************
Vector DoubleDifferencePseudorangeFactor::evaluateError(
    const Point3& pos, OptionalMatrixType H) const {
  Matrix13 H_pos;
  const double ddModel = dd_.model(pos, H ? &H_pos : nullptr);
  const double error = ddModel - dd_.observed();
  if (H) *H = H_pos;
  return Vector1(error);
}

//***************************************************************************
DoubleDifferencePseudorangeFactorArm::DoubleDifferencePseudorangeFactorArm(
    const Key poseKey, const double prRovRef, const double prBaseRef,
    const double prRovTarget, const double prBaseTarget,
    const Point3& satRefRov, const Point3& satTargetRov,
    const Point3& satRefBase, const Point3& satTargetBase,
    const Point3& basePos, const Point3& leverArm,
    const SharedNoiseModel& model)
    : Base(model, poseKey),
      dd_{prRovRef, prBaseRef, prRovTarget, prBaseTarget,
          satRefRov, satTargetRov, satRefBase, satTargetBase, basePos},
      arm_(leverArm) {}

//***************************************************************************
DoubleDifferencePseudorangeFactorArm::DoubleDifferencePseudorangeFactorArm(
    const Key poseKey, const double prRovRef, const double prBaseRef,
    const double prRovTarget, const double prBaseTarget,
    const Point3& satRefRov, const Point3& satTargetRov,
    const Point3& satRefBase, const Point3& satTargetBase,
    const Point3& basePos, const Point3& leverArm, const Pose3& ecef_T_nav,
    const SharedNoiseModel& model)
    : Base(model, poseKey),
      dd_{prRovRef, prBaseRef, prRovTarget, prBaseTarget,
          satRefRov, satTargetRov, satRefBase, satTargetBase, basePos},
      arm_(leverArm, ecef_T_nav) {}

//***************************************************************************
void DoubleDifferencePseudorangeFactorArm::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ")
            << "DoubleDifferencePseudorangeFactorArm\n";
  Base::print("", keyFormatter);
}

//***************************************************************************
bool DoubleDifferencePseudorangeFactorArm::equals(
    const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && dd_.equals(e->dd_, tol) &&
         arm_.equals(e->arm_, tol);
}

//***************************************************************************
Vector DoubleDifferencePseudorangeFactorArm::evaluateError(
    const Pose3& pose, OptionalMatrixType H_pose) const {
  gnss::LeverArm::PoseFrame frame;
  const Point3 antennaPos =
      arm_.antennaPosition(pose, H_pose ? &frame : nullptr);

  Matrix13 H_antenna;
  const double ddModel =
      dd_.model(antennaPos, H_pose ? &H_antenna : nullptr);
  const double error = ddModel - dd_.observed();

  if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_antenna, frame);
  return Vector1(error);
}

}  // namespace gtsam
