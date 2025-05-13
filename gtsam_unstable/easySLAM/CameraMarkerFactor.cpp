/**
 * @file    cameraMarkerFactor.cpp
 * @brief   A non-linear factor specialized to the EasySLAM problem
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#include "CameraMarkerFactor.h"

#include "Marker.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
void CameraMarkerFactor::init(int rn, int mn, gtsam::Cal3_S2 K,
                              int markerSize) {
  // assign identifiers to the marker (robot pose index and marker index)
  robotNumber_ = rn;
  markerNumber_ = mn;

  // create the string version of the indices and add the keys to the keys list
  char temp[100];
  temp[0] = 0;
  sprintf(temp, "x%d", robotNumber_);
  robotName_ = string(temp);
  temp[0] = 0;
  sprintf(temp, "m%d", markerNumber_);
  markerName_ = string(temp);
  list<string> keys;
  keys.push_back(robotName_);
  keys.push_back(markerName_);
  set_keys(keys);

  // initialize the calibration and markerSize
  K_ = K;
  markerSize_ = markerSize;
}

/* ************************************************************************* */
CameraMarkerFactor::CameraMarkerFactor(const Vector& z, double sigma, int rn,
                                       int mn, Cal3_S2 K, int markerSize)
    : NonlinearFactor(z, sigma) {
  init(rn, mn, K, markerSize);
}

/* ************************************************************************* */
CameraMarkerFactor::CameraMarkerFactor(const ARRobotMarker& marker,
                                       double sigma)
    : NonlinearFactor(marker.measurement(), sigma) {
  init(marker.getRobotNumber(), marker.getMarkerNumber(),
       marker.getCalibration(), marker.getMarkerSize());
}

/* ************************************************************************* */
void CameraMarkerFactor::print(const std::string& s) const {
  printf("%s %s %s\n", s.c_str(), robotName_.c_str(), markerName_.c_str());
  ::print(z_, s + ".z");
}

/* ************************************************************************* */
Vector CameraMarkerFactor::error_vector(const FGConfig& c) const {
  Pose3 pose = c[robotName_];
  Pose3 marker = c[markerName_];
  SimpleCamera camera(K_, pose);

  // Right-hand-side b = (z - h(x))/sigma
  Marker markerObj(marker, markerSize_);
  // Vector hAll = hGetAll(camera, marker, markerSize_);
  Vector hAll = hGetAll(camera, markerObj);

  return (z_ - hAll);
}

/* ************************************************************************* */
LinearFactor::shared_ptr CameraMarkerFactor::linearize(
    const FGConfig& c) const {
  // get arguments from config
  Pose3 pose = c[robotName_];
  Pose3 marker = c[markerName_];

  SimpleCamera camera(K_, pose);

  // evaluate value and derivatives all at same time
  Point2 pA, pB, pC, pD;
  Matrix DhA1, DhB1, DhC1, DhD1, DhA2, DhB2, DhC2, DhD2;
  Marker markerObj(marker, markerSize_);
  /*
    DhGetP(camera, marker, markerSize_, 1, pA, DhA1, DhA2);
    DhGetP(camera, marker, markerSize_, 2, pB, DhB1, DhB2);
    DhGetP(camera, marker, markerSize_, 3, pC, DhC1, DhC2);
    DhGetP(camera, marker, markerSize_, 4, pD, DhD1, DhD2);
  */
  DhGetP(camera, markerObj, 1, pA, DhA1, DhA2);
  DhGetP(camera, markerObj, 2, pB, DhB1, DhB2);
  DhGetP(camera, markerObj, 3, pC, DhC1, DhC2);
  DhGetP(camera, markerObj, 4, pD, DhD1, DhD2);

  // Jacobian A = H(x)
  Matrix Dh1All = stack(4, &DhA1, &DhB1, &DhC1, &DhD1);
  Matrix Dh2All = stack(4, &DhA2, &DhB2, &DhC2, &DhD2);

  // Right-hand-side b = (z - h(x))
  Vector hA = pA.vector(), hB = pB.vector(), hC = pC.vector(), hD = pD.vector();
  Vector b = z_ - concatVectors(4, &hA, &hB, &hC, &hD);

  // Make new linearfactor, divide by sigma
  LinearFactor::shared_ptr p(new LinearFactor(
      robotName_, Dh1All / sigma_, markerName_, Dh2All / sigma_, b / sigma_));
  return p;
}

/* ************************************************************************* */
bool CameraMarkerFactor::equals(const Factor& f, double tol) const {
  const CameraMarkerFactor* p = dynamic_cast<const CameraMarkerFactor*>(&f);
  if (p == NULL) goto fail;
  // if (!NonlinearFactor::equals(*p,tol)) goto fail;
  if (robotNumber_ != p->robotNumber_ || markerNumber_ != p->markerNumber_)
    goto fail;
  if (!equal_with_abs_tol(z_, p->z_, tol)) goto fail;
  return true;

fail:
  print("actual");
  p->print("expected");
  return false;
}

/* ************************************************************************* */
string CameraMarkerFactor::dump() const {
  // printf("CameraMarkerFactor\n");
  int i = getRobotNumber();
  int j = getMarkerNumber();
  double sigma = get_sigma();
  Vector z = get_measurement();
  char buffer[200];
  buffer[0] = 0;
  sprintf(buffer, "1 %d %d %f %d", i, j, sigma, z.size());
  for (size_t i = 0; i < z.size(); i++) sprintf(buffer, "%s %f", buffer, z(i));
  sprintf(buffer, "%s %s", buffer, K_.dump().c_str());
  sprintf(buffer, "%s %f", buffer, markerSize_);

  return string(buffer);
}
