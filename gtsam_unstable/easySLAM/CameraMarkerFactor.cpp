/**
 * @file    CombinedCameraMarkerFactor.cpp
 * @brief   Implementations for non-linear factors specialized to the EasySLAM problem.
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#include "CameraMarkerFactor.h"
#include "Marker.h" // Assumed to provide Marker class, hGetAll, DhGetP

 // GTSAM related includes
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/linear/LinearFactor.h>
#include <gtsam/nonlinear/FGConfig.h> // For gtsam::Values (via FGConfig typedef)
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/LieVector.h>      // For gtsam::print(Vector)
#include <gtsam/base/utilities.h>    // For gtsam::equal_with_abs_tol, stack, concatVectors

// Standard library includes
#include <cstdio> // For sprintf
#include <string>
#include <list>
// #include <sstream> // For safer string formatting if fixing dump()

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Implementations for CameraMarkerFactor
/* ************************************************************************* */
void CameraMarkerFactor::init(int rn, int mn, gtsam::Cal3_S2 K,
  int markerSize) {
  robotNumber_ = rn;
  markerNumber_ = mn;

  char temp[100];
  sprintf(temp, "x%d", robotNumber_);
  robotName_ = string(temp);
  sprintf(temp, "m%d", markerNumber_);
  markerName_ = string(temp);

  list<string> keys;
  keys.push_back(robotName_);
  keys.push_back(markerName_);
  set_keys(keys); // Method from gtsam::Factor base

  K_ = K;
  markerSize_ = static_cast<double>(markerSize);
}

CameraMarkerFactor::CameraMarkerFactor(const Vector& z, double sigma, int rn,
  int mn, Cal3_S2 K, int markerSize)
  : NonlinearFactor(z, sigma) { // Assuming this base constructor is available
  init(rn, mn, K, markerSize);
}

CameraMarkerFactor::CameraMarkerFactor(const ARRobotMarker& marker,
  double sigma)
  : NonlinearFactor(marker.measurement(), sigma) { // Assuming this base constructor
  init(marker.getRobotNumber(), marker.getMarkerNumber(),
    marker.getCalibration(), marker.getMarkerSize());
}

void CameraMarkerFactor::print(const std::string& s) const {
  printf("%s (keys: %s, %s)\n", s.c_str(), robotName_.c_str(), markerName_.c_str());
  gtsam::print(z_, s + ".z"); // z_ is assumed from custom NonlinearFactor base
}

Vector CameraMarkerFactor::error_vector(const FGConfig& c) const {
  Pose3 pose = c.at<Pose3>(robotName_);
  Pose3 marker_pose = c.at<Pose3>(markerName_);
  SimpleCamera camera(K_, pose);

  Marker markerObj(marker_pose, markerSize_);
  Vector hAll = hGetAll(camera, markerObj); // Assumes hGetAll is defined in Marker.h context

  return (z_ - hAll);
}

LinearFactor::shared_ptr CameraMarkerFactor::linearize(
  const FGConfig& c) const {
  Pose3 pose = c.at<Pose3>(robotName_);
  Pose3 marker_pose = c.at<Pose3>(markerName_);
  SimpleCamera camera(K_, pose);

  Point2 pA, pB, pC, pD;
  Matrix DhA1, DhB1, DhC1, DhD1; // Jacobians wrt pose
  Matrix DhA2, DhB2, DhC2, DhD2; // Jacobians wrt marker_pose
  Marker markerObj(marker_pose, markerSize_);

  DhGetP(camera, markerObj, 1, pA, DhA1, DhA2); // Assumes DhGetP is defined
  DhGetP(camera, markerObj, 2, pB, DhB1, DhB2);
  DhGetP(camera, markerObj, 3, pC, DhC1, DhC2);
  DhGetP(camera, markerObj, 4, pD, DhD1, DhD2);

  Matrix Dh1All = stack(4, &DhA1, &DhB1, &DhC1, &DhD1);
  Matrix Dh2All = stack(4, &DhA2, &DhB2, &DhC2, &DhD2);

  Vector hA = pA.vector(), hB = pB.vector(), hC = pC.vector(), hD = pD.vector();
  Vector b = z_ - concatVectors(4, &hA, &hB, &hC, &hD);

  double sigma_val = get_sigma(); // From Factor base class (noiseModel()->sigma())

  return LinearFactor::shared_ptr(new LinearFactor(
    robotName_, Dh1All / sigma_val, markerName_, Dh2All / sigma_val, b / sigma_val));
}

bool CameraMarkerFactor::equals(const Factor& f, double tol) const {
  const CameraMarkerFactor* p = dynamic_cast<const CameraMarkerFactor*>(&f);
  if (p == NULL) goto fail; // Using goto as in original
  // if (!NonlinearFactor::equals(*p,tol)) goto fail; // Original was commented
  if (robotNumber_ != p->robotNumber_ || markerNumber_ != p->markerNumber_)
    goto fail;
  if (!equal_with_abs_tol(z_, p->z_, tol)) goto fail;
  // Note: K_ and markerSize_ are not compared in original.
  return true;

fail:
  this->print("actual: CameraMarkerFactor::equals");
  if (p) p->print("expected: CameraMarkerFactor::equals");
  else printf("expected: CameraMarkerFactor::equals (null or wrong type)\n");
  return false;
}

std::string CameraMarkerFactor::dump() const {
  char buffer[256]; // Slightly larger buffer, still risky
  sprintf(buffer, "1 %d %d %f %zu", robotNumber_, markerNumber_, get_sigma(), z_.size());

  // THIS IS THE PROBLEMATIC SPRINTF LOOP - Replicating original structure
  // For robust code, this should be rewritten (e.g. using std::ostringstream)
  for (size_t k = 0; k < z_.size(); k++) {
    char temp_num[50];
    sprintf(temp_num, " %f", z_(k));
    strncat(buffer, temp_num, sizeof(buffer) - strlen(buffer) - 1);
  }

  std::string k_dump_str = K_.dump(); // Assuming K_.dump() returns std::string
  char temp_k[256]; // Assuming K dump is not excessively long
  sprintf(temp_k, " %s %f", k_dump_str.c_str(), markerSize_);
  strncat(buffer, temp_k, sizeof(buffer) - strlen(buffer) - 1);

  return std::string(buffer);
}

/* ************************************************************************* */
// Implementations for CameraMarkerFactor0
/* ************************************************************************* */
CameraMarkerFactor0::CameraMarkerFactor0(const Vector& z, double sigma, int rn,
  Cal3_S2 K, int markerSize,
  const Pose3& knownMarker)
  : NonlinearFactor(z, sigma), marker_(knownMarker) {
  robotNumber_ = rn;

  char temp[100];
  sprintf(temp, "x%d", robotNumber_);
  robotName_ = string(temp);

  K_ = K;
  markerSize_ = static_cast<double>(markerSize);

  list<string> keys;
  keys.push_back(robotName_);
  set_keys(keys);
}

CameraMarkerFactor0::CameraMarkerFactor0(const ARRobotMarker& marker_data, double sigma,
  const Pose3& knownMarker)
  : NonlinearFactor(marker_data.measurement(), sigma), marker_(knownMarker) {
  robotNumber_ = marker_data.getRobotNumber();

  char temp[100];
  sprintf(temp, "x%d", robotNumber_);
  robotName_ = string(temp);

  K_ = marker_data.getCalibration();
  markerSize_ = static_cast<double>(marker_data.getMarkerSize());

  list<string> keys;
  keys.push_back(robotName_);
  set_keys(keys);
}

void CameraMarkerFactor0::print(const std::string& s) const {
  printf("%s (key: %s)\n", s.c_str(), robotName_.c_str());
  gtsam::print(z_, s + ".z");
}

Vector CameraMarkerFactor0::error_vector(const FGConfig& c) const {
  Pose3 pose = c.at<Pose3>(robotName_); // Robot pose to be estimated
  SimpleCamera camera(K_, pose);

  // marker_ is the known fixed pose of the marker
  Marker markerObj(marker_, markerSize_);
  Vector hAll = hGetAll(camera, markerObj);

  return (z_ - hAll);
}

LinearFactor::shared_ptr CameraMarkerFactor0::linearize(
  const FGConfig& c) const {
  Pose3 pose = c.at<Pose3>(robotName_);
  SimpleCamera camera(K_, pose);

  Point2 pA, pB, pC, pD;
  Matrix DhA1, DhB1, DhC1, DhD1; // Derivatives wrt robot pose
  Matrix DhA2, DhB2, DhC2, DhD2; // Derivatives wrt marker pose (ignored for Jacobian)

  Marker markerObj(marker_, markerSize_);
  DhGetP(camera, markerObj, 1, pA, DhA1, DhA2);
  DhGetP(camera, markerObj, 2, pB, DhB1, DhB2);
  DhGetP(camera, markerObj, 3, pC, DhC1, DhC2);
  DhGetP(camera, markerObj, 4, pD, DhD1, DhD2);

  Matrix Dh1All = stack(4, &DhA1, &DhB1, &DhC1, &DhD1);

  Vector hA = pA.vector(), hB = pB.vector(), hC = pC.vector(), hD = pD.vector();
  Vector b = z_ - concatVectors(4, &hA, &hB, &hC, &hD);

  double sigma_val = get_sigma();
  return LinearFactor::shared_ptr(
    new LinearFactor(robotName_, Dh1All / sigma_val, b / sigma_val));
}

bool CameraMarkerFactor0::equals(const Factor& f, double tol) const {
  const CameraMarkerFactor0* p = dynamic_cast<const CameraMarkerFactor0*>(&f);
  if (p == NULL) goto fail;
  // if (!NonlinearFactor::equals(*p,tol)) goto fail;
  if (robotNumber_ != p->robotNumber_) goto fail;
  if (!equal_with_abs_tol(z_, p->z_, tol)) goto fail;
  // Note: K_, markerSize_, marker_ (knownMarkerPose) are not compared.
  return true;

fail:
  this->print("actual: CameraMarkerFactor0::equals");
  if (p) p->print("expected: CameraMarkerFactor0::equals");
  else printf("expected: CameraMarkerFactor0::equals (null or wrong type)\n");
  return false;
}

std::string CameraMarkerFactor0::dump() const {
  char buffer[256]; // Risky fixed-size buffer
  sprintf(buffer, "0 %d %f %zu", robotNumber_, get_sigma(), z_.size());

  // Replicating original problematic sprintf loop for appending numbers
  for (size_t k = 0; k < z_.size(); k++) {
    char temp_num[50];
    sprintf(temp_num, " %f", z_(k));
    strncat(buffer, temp_num, sizeof(buffer) - strlen(buffer) - 1);
  }

  std::string k_dump_str = K_.dump();
  char temp_k[256];
  sprintf(temp_k, " %s %f", k_dump_str.c_str(), markerSize_);
  strncat(buffer, temp_k, sizeof(buffer) - strlen(buffer) - 1);
  // Note: known marker pose (marker_) is not dumped.

  return std::string(buffer);
}


/* ************************************************************************* */
// Implementations for CameraMarkerFactor1
/* ************************************************************************* */
CameraMarkerFactor1::CameraMarkerFactor1(const Vector& z, double sigma, int mn,
  Cal3_S2 K, int markerSize,
  const Pose3& knownRobot)
  : NonlinearFactor(z, sigma), robotPose_(knownRobot) {
  markerNumber_ = mn;

  char temp[100];
  sprintf(temp, "m%d", markerNumber_);
  markerName_ = string(temp); // Key for the marker to be estimated

  K_ = K;
  markerSize_ = static_cast<double>(markerSize);

  list<string> keys;
  keys.push_back(markerName_);
  set_keys(keys);
}

CameraMarkerFactor1::CameraMarkerFactor1(const ARRobotMarker& marker_data, double sigma,
  const Pose3& knownRobot)
  : NonlinearFactor(marker_data.measurement(), sigma), robotPose_(knownRobot) {
  markerNumber_ = marker_data.getMarkerNumber();

  char temp[100];
  sprintf(temp, "m%d", markerNumber_);
  markerName_ = string(temp);

  K_ = marker_data.getCalibration();
  markerSize_ = static_cast<double>(marker_data.getMarkerSize());

  list<string> keys;
  keys.push_back(markerName_);
  set_keys(keys);
}

void CameraMarkerFactor1::print(const std::string& s) const {
  printf("%s (key: %s)\n", s.c_str(), markerName_.c_str());
  gtsam::print(z_, s + ".z");
}

Vector CameraMarkerFactor1::error_vector(const FGConfig& c) const {
  Pose3 marker_pose = c.at<Pose3>(markerName_); // Marker pose to be estimated

  // Camera is at robotPose_ (which is knownRobot).
  // ORIGINAL CODE HAD: Pose3 camera_pose_at_origin; SimpleCamera camera(K_, camera_pose_at_origin);
  // This seemed like a bug if robotPose_ could be non-origin.
  // Assuming the intent is to use the known robot pose for the camera:
  SimpleCamera camera(K_, robotPose_);
  // If the original intent was strictly camera at origin, then use:
  // Pose3 camera_pose_at_origin; SimpleCamera camera(K_, camera_pose_at_origin);

  Marker markerObj(marker_pose, markerSize_);
  Vector hAll = hGetAll(camera, markerObj);

  return (z_ - hAll);
}

LinearFactor::shared_ptr CameraMarkerFactor1::linearize(
  const FGConfig& c) const {
  Pose3 marker_pose = c.at<Pose3>(markerName_);

  // Consistent with error_vector: camera at robotPose_
  SimpleCamera camera(K_, robotPose_);
  // If original camera at origin behavior is required:
  // Pose3 camera_pose_at_origin; SimpleCamera camera(K_, camera_pose_at_origin);

  Point2 pA, pB, pC, pD;
  Matrix DhA1, DhB1, DhC1, DhD1; // Derivatives wrt camera pose (not used for Jacobian)
  Matrix DhA2, DhB2, DhC2, DhD2; // Derivatives wrt marker pose

  Marker markerObj(marker_pose, markerSize_);
  DhGetP(camera, markerObj, 1, pA, DhA1, DhA2);
  DhGetP(camera, markerObj, 2, pB, DhB1, DhB2);
  DhGetP(camera, markerObj, 3, pC, DhC1, DhC2);
  DhGetP(camera, markerObj, 4, pD, DhD1, DhD2);

  Matrix Dh_wrt_marker_All = stack(4, &DhA2, &DhB2, &DhC2, &DhD2);

  Vector hA = pA.vector(), hB = pB.vector(), hC = pC.vector(), hD = pD.vector();
  Vector b = z_ - concatVectors(4, &hA, &hB, &hC, &hD);

  double sigma_val = get_sigma();
  return LinearFactor::shared_ptr(
    new LinearFactor(markerName_, Dh_wrt_marker_All / sigma_val, b / sigma_val));
}

bool CameraMarkerFactor1::equals(const Factor& f, double tol) const {
  const CameraMarkerFactor1* p = dynamic_cast<const CameraMarkerFactor1*>(&f);
  if (p == NULL) goto fail;
  // if (!NonlinearFactor::equals(*p,tol)) goto fail;
  if (markerNumber_ != p->markerNumber_) goto fail;
  if (!equal_with_abs_tol(z_, p->z_, tol)) goto fail;
  // Note: K_, markerSize_, robotPose_ (knownRobotPose) are not compared.
  return true;

fail:
  this->print("actual: CameraMarkerFactor1::equals");
  if (p) p->print("expected: CameraMarkerFactor1::equals");
  else printf("expected: CameraMarkerFactor1::equals (null or wrong type)\n");
  return false;
}

std::string CameraMarkerFactor1::dump() const {
  char buffer[256]; // Risky fixed-size buffer
  // Original dump format for Factor1 started with "0", same as Factor0.
  sprintf(buffer, "0 %d %f %zu", markerNumber_, get_sigma(), z_.size());

  // Replicating original problematic sprintf loop
  for (size_t k = 0; k < z_.size(); k++) {
    char temp_num[50];
    sprintf(temp_num, " %f", z_(k));
    strncat(buffer, temp_num, sizeof(buffer) - strlen(buffer) - 1);
  }

  std::string k_dump_str = K_.dump();
  char temp_k[256];
  sprintf(temp_k, " %s %f", k_dump_str.c_str(), markerSize_);
  strncat(buffer, temp_k, sizeof(buffer) - strlen(buffer) - 1);
  // Note: known robot pose (robotPose_) is not dumped.

  return std::string(buffer);
}