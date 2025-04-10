/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GPSFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for GPS factor
 *  @date   January 28, 2014
 **/

#include "GPSFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void GPSFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "GPSFactor on " << keyFormatter(key())
       << "\n";
  cout << "  GPS measurement: " << nT_ << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(nT_, e->nT_, tol);
}

//***************************************************************************
Vector GPSFactor::evaluateError(const Pose3& nTb,
    OptionalMatrixType H) const {
  return nTb.translation(H) -nT_;
}

//***************************************************************************
pair<Pose3, Vector3> GPSFactor::EstimateState(double t1, const Point3& NED1,
    double t2, const Point3& NED2, double timestamp) {
  // Estimate initial velocity as difference in NED frame
  double dt = t2 - t1;
  Point3 nV = (NED2 - NED1) / dt;

  // Estimate initial position as linear interpolation
  Point3 nT = NED1 + nV * (timestamp - t1);

  // Estimate Rotation
  double yaw = atan2(nV.y(), nV.x());
  Rot3 nRy = Rot3::Yaw(yaw); // yaw frame
  Point3 yV = nRy.inverse() * nV; // velocity in yaw frame
  double pitch = -atan2(yV.z(), yV.x()), roll = 0;
  Rot3 nRb = Rot3::Ypr(yaw, pitch, roll);

  // Construct initial pose
  Pose3 nTb(nRb, nT); // nTb

  return make_pair(nTb, nV);
}

//***************************************************************************
void GPSFactorArm::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "GPSFactorArm on " << keyFormatter(key()) << "\n";
  cout << "  GPS measurement: " << nT_.transpose() << "\n";
  cout << "  Lever arm: " << bL_.transpose() << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactorArm::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol) &&
         traits<Point3>::Equals(bL_, e->bL_, tol);
}

//***************************************************************************
Vector GPSFactorArm::evaluateError(const Pose3& nTb,
    OptionalMatrixType H) const {
  const Matrix3 nRb = nTb.rotation().matrix();
  if (H) {
    H->resize(3, 6);

    H->block<3, 3>(0, 0) = -nRb * skewSymmetric(bL_);
    H->block<3, 3>(0, 3) = nRb;
  }

  return nTb.translation() + nRb * bL_ - nT_;
}

//***************************************************************************
void GPSFactorArmCalib::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "GPSFactorArmCalib on " << keyFormatter(key()) << "\n";
  cout << "  GPS measurement: " << nT_.transpose() << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactorArmCalib::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol);
}

//***************************************************************************
Vector GPSFactorArmCalib::evaluateError(const Pose3& nTb, const Point3& bL,
    OptionalMatrixType H1, OptionalMatrixType H2) const {
  const Matrix3 nRb = nTb.rotation().matrix();
  if (H1) {
    H1->resize(3, 6);

    H1->block<3, 3>(0, 0) = -nRb * skewSymmetric(bL);
    H1->block<3, 3>(0, 3) = nRb;
  }
  if (H2){
    H2->resize(3, 3);
    *H2 = nRb;
  }

  return nTb.translation() + nRb * bL - nT_;
}

//***************************************************************************
void GPSFactor2::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "GPSFactor2 on " << keyFormatter(key()) << "\n";
  cout << "  GPS measurement: " << nT_.transpose() << endl;
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactor2::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol);
}

//***************************************************************************
Vector GPSFactor2::evaluateError(const NavState& nTb,
    OptionalMatrixType H) const {
  return nTb.position(H) -nT_;
}

//***************************************************************************
void GPSFactor2Arm::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "GPSFactor2Arm on " << keyFormatter(key()) << "\n";
  cout << "  GPS measurement: " << nT_.transpose() << "\n";
  cout << "  Lever arm: " << bL_.transpose() << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactor2Arm::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol) &&
         traits<Point3>::Equals(bL_, e->bL_, tol);
}

//***************************************************************************
Vector GPSFactor2Arm::evaluateError(const NavState& nTb,
    OptionalMatrixType H) const {
  const Matrix3 nRb = nTb.attitude().matrix();
  if (H) {
    H->resize(3, 9);

    H->block<3, 3>(0, 0) = -nRb * skewSymmetric(bL_);
    H->block<3, 3>(0, 3) = nRb;
    H->block<3, 3>(0, 6).setZero();
  }

  return nTb.position() + nRb * bL_ - nT_;
}

//***************************************************************************
void GPSFactor2ArmCalib::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "GPSFactor2ArmCalib on " << keyFormatter(key()) << "\n";
  cout << "  GPS measurement: " << nT_.transpose() << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactor2ArmCalib::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol);
}

//***************************************************************************
Vector GPSFactor2ArmCalib::evaluateError(const NavState& nTb, const Point3& bL,
    OptionalMatrixType H1, OptionalMatrixType H2) const {
  const Matrix3 nRb = nTb.attitude().matrix();
  if (H1) {
    H1->resize(3, 9);

    H1->block<3, 3>(0, 0) = -nRb * skewSymmetric(bL);
    H1->block<3, 3>(0, 3) = nRb;
    H1->block<3, 3>(0, 6).setZero();
  }
  if (H2){
    H2->resize(3, 3);
    *H2 = nRb;
  }

  return nTb.position() + nRb * bL - nT_;
}

}/// namespace gtsam
