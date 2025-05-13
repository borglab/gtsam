/**
 * @file    cameraMarkerFactor.cpp
 * @brief   A non-linear factor specialized to the EasySLAM problem
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#include "CameraMarkerFactor1.h"

#include "Marker.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
CameraMarkerFactor1::CameraMarkerFactor1(const Vector& z, double sigma, int mn,
                                         Cal3_S2 K, int markerSize,
                                         const Pose3& knownRobot)
    : NonlinearFactor(z, sigma), marker_(knownRobot) {
  markerNumber_ = mn;

  char temp[100];
  temp[0] = 0;
  sprintf(temp, "m%d", markerNumber_);
  markerName_ = string(temp);
  K_ = K;
  markerSize_ = markerSize;
  list<string> keys;
  keys.push_back(markerName_);
  set_keys(keys);
}

/* ************************************************************************* */
CameraMarkerFactor1::CameraMarkerFactor1(const ARRobotMarker& marker,
                                         double sigma, const Pose3& knownRobot)
    : NonlinearFactor(marker.measurement(), sigma), marker_(knownRobot) {
  markerNumber_ = marker.getMarkerNumber();
  char temp[100];
  temp[0] = 0;
  sprintf(temp, "m%d", markerNumber_);
  markerName_ = string(temp);
  K_ = marker.getCalibration();
  markerSize_ = marker.getMarkerSize();
  list<string> keys;
  keys.push_back(markerName_);
  set_keys(keys);
}

/* ************************************************************************* */
void CameraMarkerFactor1::print(const std::string& s) const {
  printf("%s %s\n", s.c_str(), markerName_.c_str());
  ::print(z_, s + ".z");
}

/* ************************************************************************* */
Vector CameraMarkerFactor1::error_vector(const FGConfig& c) const {
  Pose3 marker = c[markerName_];
  Pose3 origin;

  SimpleCamera cameraAtOrigin(K_, origin);

  // Right-hand-side b = (z - h(x))/sigma
  Marker markerObj(marker, markerSize_);
  // Vector hAll = hGetAll(cameraAtOrigin, marker, markerSize_);
  Vector hAll = hGetAll(cameraAtOrigin, markerObj);

  return (z_ - hAll);
}

/* ************************************************************************* */
LinearFactor::shared_ptr CameraMarkerFactor1::linearize(
    const FGConfig& c) const {
  // get arguments from config
  Pose3 marker = c[markerName_];
  Pose3 origin;

  SimpleCamera cameraAtOrigin(K_, origin);

  // evaluate value and derivatives all at same time
  Point2 pA, pB, pC, pD;
  Matrix DhA1, DhB1, DhC1, DhD1, DhA2, DhB2, DhC2, DhD2;
  Marker markerObj(marker, markerSize_);
  /*DhGetP(cameraAtOrigin, marker, markerSize_, 1, pA, DhA1, DhA2);
  DhGetP(cameraAtOrigin, marker, markerSize_, 2, pB, DhB1, DhB2);
  DhGetP(cameraAtOrigin, marker, markerSize_, 3, pC, DhC1, DhC2);
  DhGetP(cameraAtOrigin, marker, markerSize_, 4, pD, DhD1, DhD2);
*/
  DhGetP(cameraAtOrigin, markerObj, 1, pA, DhA1, DhA2);
  DhGetP(cameraAtOrigin, markerObj, 2, pB, DhB1, DhB2);
  DhGetP(cameraAtOrigin, markerObj, 3, pC, DhC1, DhC2);
  DhGetP(cameraAtOrigin, markerObj, 4, pD, DhD1, DhD2);

  // Jacobian A = H(x)
  Matrix Dh1All = stack(4, &DhA2, &DhB2, &DhC2, &DhD2);

  // Right-hand-side b = (z - h(x))
  Vector hA = pA.vector(), hB = pB.vector(), hC = pC.vector(), hD = pD.vector();
  Vector b = z_ - concatVectors(4, &hA, &hB, &hC, &hD);

  // Make new linearfactor, divide by sigma
  LinearFactor::shared_ptr p(
      new LinearFactor(markerName_, Dh1All / sigma_, b / sigma_));
  return p;
}

/* ************************************************************************* */
bool CameraMarkerFactor1::equals(const Factor& f, double tol) const {
  const CameraMarkerFactor1* p = dynamic_cast<const CameraMarkerFactor1*>(&f);
  if (p == NULL) goto fail;
  // if (!NonlinearFactor::equals(*p,tol)) goto fail;
  if (markerNumber_ != p->markerNumber_) goto fail;
  if (!equal_with_abs_tol(z_, p->z_, tol)) goto fail;
  return true;

fail:
  print("actual");
  p->print("expected");
  return false;
}

/* ************************************************************************* */
string CameraMarkerFactor1::dump() const {
  // printf("CameraMarkerFactor0\n");
  int i = getMarkerNumber();
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
