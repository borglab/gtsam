/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SmartFactorScenarios.h
 *  @brief Scenarios for testSmart*.cpp
 *  @author Frank Dellaert
 *  @date   Feb 2015
 */

#pragma once
#include <gtsam/slam/SmartProjectionCameraFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>

using namespace std;
using namespace gtsam;

// three landmarks ~5 meters infront of camera
Point3 landmark1(5, 0.5, 1.2);
Point3 landmark2(5, -0.5, 1.2);
Point3 landmark3(3, 0, 3.0);
Point3 landmark4(10, 0.5, 1.2);
Point3 landmark5(10, -0.5, 1.2);

// First camera pose, looking along X-axis, 1 meter above ground plane (x-y)
Pose3 level_pose = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
// Second camera 1 meter to the right of first camera
Pose3 pose_right = level_pose * Pose3(Rot3(), Point3(1, 0, 0));
// Third camera 1 meter above the first camera
Pose3 pose_up = level_pose * Pose3(Rot3(), Point3(0, -1, 0));

// Create a noise unit2 for the pixel error
static SharedNoiseModel unit2(noiseModel::Unit::Create(2));

/* ************************************************************************* */
// default Cal3_S2 cameras
namespace vanilla {
typedef PinholeCamera<Cal3_S2> Camera;
// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w = 640, h = 480;
static Cal3_S2 K(fov, w, h);
static Cal3_S2 K2(1500, 1200, 0, w, h);
Camera level_camera(level_pose, K2);
Camera level_camera_right(pose_right, K2);
Point2 level_uv = level_camera.project(landmark1);
Point2 level_uv_right = level_camera_right.project(landmark1);
Camera cam1(level_pose, K2);
Camera cam2(pose_right, K2);
Camera cam3(pose_up, K2);
typedef GeneralSFMFactor<Camera, Point3> SFMFactor;
typedef SmartProjectionCameraFactor<Cal3_S2> SmartFactor;
}

/* *************************************************************************/
// Cal3Bundler cameras
namespace bundler {
typedef PinholeCamera<Cal3Bundler> Camera;
static Cal3Bundler K(500, 1e-3, 1e-3, 0, 0);
Camera level_camera(level_pose, K);
Camera level_camera_right(pose_right, K);
Point2 level_uv = level_camera.project(landmark1);
Point2 level_uv_right = level_camera_right.project(landmark1);
Pose3 pose1 = level_pose;
Camera cam1(level_pose, K);
Camera cam2(pose_right, K);
Camera cam3(pose_up, K);
typedef GeneralSFMFactor<Camera, Point3> SFMFactor;
typedef SmartProjectionCameraFactor<Cal3Bundler> SmartFactorBundler;
}
/* *************************************************************************/

template<class CALIBRATION>
PinholeCamera<CALIBRATION> perturbCameraPose(
    const PinholeCamera<CALIBRATION>& camera) {
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Pose3 cameraPose = camera.pose();
  Pose3 perturbedCameraPose = cameraPose.compose(noise_pose);
  return PinholeCamera<CALIBRATION>(perturbedCameraPose, camera.calibration());
}

template<class CALIBRATION>
PinholeCamera<CALIBRATION> perturbCameraPoseAndCalibration(
    const PinholeCamera<CALIBRATION>& camera) {
  GTSAM_CONCEPT_MANIFOLD_TYPE(CALIBRATION)
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Pose3 cameraPose = camera.pose();
  Pose3 perturbedCameraPose = cameraPose.compose(noise_pose);
  typename gtsam::traits<CALIBRATION>::TangentVector d;
  d.setRandom();
  d *= 0.1;
  CALIBRATION perturbedCalibration = camera.calibration().retract(d);
  return PinholeCamera<CALIBRATION>(perturbedCameraPose, perturbedCalibration);
}

template<class CALIBRATION>
void projectToMultipleCameras(const PinholeCamera<CALIBRATION>& cam1,
    const PinholeCamera<CALIBRATION>& cam2,
    const PinholeCamera<CALIBRATION>& cam3, Point3 landmark,
    vector<Point2>& measurements_cam) {
  Point2 cam1_uv1 = cam1.project(landmark);
  Point2 cam2_uv1 = cam2.project(landmark);
  Point2 cam3_uv1 = cam3.project(landmark);
  measurements_cam.push_back(cam1_uv1);
  measurements_cam.push_back(cam2_uv1);
  measurements_cam.push_back(cam3_uv1);
}

/* ************************************************************************* */

