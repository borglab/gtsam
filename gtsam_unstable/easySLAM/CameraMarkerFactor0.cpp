/**
 * @file    cameraMarkerFactor.cpp
 * @brief   A non-linear factor specialized to the EasySLAM problem
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#include "CameraMarkerFactor0.h"

#include "Marker.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
CameraMarkerFactor0::CameraMarkerFactor0(const Vector& z, double sigma, int rn,
                                         Cal3_S2 K, int markerSize,
                                         const Pose3& knownMarker)
    : NonlinearFactor(z, sigma), marker_(knownMarker) {
  robotNumber_ = rn;

  char temp[100];
  temp[0] = 0;
  sprintf(temp, "x%d", robotNumber_);
  robotName_ = string(temp);
  K_ = K;
  markerSize_ = markerSize;
  list<string> keys;
  keys.push_back(robotName_);
  set_keys(keys);
}

/* ************************************************************************* */
CameraMarkerFactor0::CameraMarkerFactor0(const ARRobotMarker& marker,
                                         double sigma, const Pose3& knownMarker)
    : NonlinearFactor(marker.measurement(), sigma), marker_(knownMarker) {
  robotNumber_ = marker.getRobotNumber();
  char temp[100];
  temp[0] = 0;
  sprintf(temp, "x%d", robotNumber_);
  robotName_ = string(temp);
  K_ = marker.getCalibration();
  markerSize_ = marker.getMarkerSize();
  list<string> keys;
  keys.push_back(robotName_);
  set_keys(keys);
}

/* ************************************************************************* */
void CameraMarkerFactor0::print(const std::string& s) const {
  printf("%s %s\n", s.c_str(), robotName_.c_str());
  ::print(z_, s + ".z");
}

/* ************************************************************************* */
Vector CameraMarkerFactor0::error_vector(const FGConfig& c) const {
  Pose3 pose = c[robotName_];

  SimpleCamera camera(K_, pose);

  // Right-hand-side b = (z - h(x))/sigma
  // TODO: hGetP could be simplified in this case since we don't have the marker
  // anymore
  Marker markerObj(marker_, markerSize_);
  // Vector hAll = hGetAll(camera, marker_, markerSize_);
  Vector hAll = hGetAll(camera, markerObj);

  return (z_ - hAll);
}

/* ************************************************************************* */
LinearFactor::shared_ptr CameraMarkerFactor0::linearize(
    const FGConfig& c) const {
  // get arguments from config
  Pose3 pose = c[robotName_];

  SimpleCamera camera(K_, pose);

  // evaluate value and derivatives all at same time
  // TODO: here we don't care about the derivatives with respect to markers
  // (Dh*2)
  Point2 pA, pB, pC, pD;
  Matrix DhA1, DhB1, DhC1, DhD1, DhA2, DhB2, DhC2, DhD2;
  Marker markerObj(marker_, markerSize_);
  /*
    DhGetP(camera, marker_, markerSize_, 1, pA, DhA1, DhA2);
    DhGetP(camera, marker_, markerSize_, 2, pB, DhB1, DhB2);
    DhGetP(camera, marker_, markerSize_, 3, pC, DhC1, DhC2);
    DhGetP(camera, marker_, markerSize_, 4, pD, DhD1, DhD2);
  */
  DhGetP(camera, markerObj, 1, pA, DhA1, DhA2);
  DhGetP(camera, markerObj, 2, pB, DhB1, DhB2);
  DhGetP(camera, markerObj, 3, pC, DhC1, DhC2);
  DhGetP(camera, markerObj, 4, pD, DhD1, DhD2);

  // Jacobian A = H(x)
  Matrix Dh1All = stack(4, &DhA1, &DhB1, &DhC1, &DhD1);

  // Right-hand-side b = (z - h(x))
  Vector hA = pA.vector(), hB = pB.vector(), hC = pC.vector(), hD = pD.vector();
  Vector b = z_ - concatVectors(4, &hA, &hB, &hC, &hD);

  // Make new linearfactor, divide by sigma
  LinearFactor::shared_ptr p(
      new LinearFactor(robotName_, Dh1All / sigma_, b / sigma_));
  return p;
}

/* ************************************************************************* */
bool CameraMarkerFactor0::equals(const Factor& f, double tol) const {
  const CameraMarkerFactor0* p = dynamic_cast<const CameraMarkerFactor0*>(&f);
  if (p == NULL) goto fail;
  // if (!NonlinearFactor::equals(*p,tol)) goto fail;
  if (robotNumber_ != p->robotNumber_) goto fail;
  if (!equal_with_abs_tol(z_, p->z_, tol)) goto fail;
  return true;

fail:
  print("actual");
  p->print("expected");
  return false;
}

/* ************************************************************************* */
string CameraMarkerFactor0::dump() const {
  // printf("CameraMarkerFactor0\n");
  int i = getRobotNumber();
  double sigma = get_sigma();
  Vector z = get_measurement();
  char buffer[200];
  buffer[0] = 0;
  sprintf(buffer, "0 %d %f %d", i, sigma, z.size());
  for (size_t i = 0; i < z.size(); i++) sprintf(buffer, "%s %f", buffer, z(i));
  sprintf(buffer, "%s %s", buffer, K_.dump().c_str());
  sprintf(buffer, "%s %f", buffer, markerSize_);

  return string(buffer);
}
