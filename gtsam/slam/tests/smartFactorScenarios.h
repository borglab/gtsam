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
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include "../SmartProjectionRigFactor.h"

using namespace std;
using namespace gtsam;

// three landmarks ~5 meters infront of camera
Point3 landmark1(5, 0.5, 1.2);
Point3 landmark2(5, -0.5, 1.2);
Point3 landmark3(3, 0, 3.0);
Point3 landmark4(10, 0.5, 1.2);
Point3 landmark5(10, -0.5, 1.2);

// First camera pose, looking along X-axis, 1 meter above ground plane (x-y)
Pose3 level_pose = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
// Second camera 1 meter to the right of first camera
Pose3 pose_right = level_pose * Pose3(Rot3(), Point3(1, 0, 0));
// Third camera 1 meter above the first camera
Pose3 pose_above = level_pose * Pose3(Rot3(), Point3(0, -1, 0));

// Create a noise unit2 for the pixel error
static SharedNoiseModel unit2(noiseModel::Unit::Create(2));

static double fov = 60; // degrees
static size_t w = 640, h = 480;

/* ************************************************************************* */
// default Cal3_S2 cameras
namespace vanilla {
typedef PinholeCamera<Cal3_S2> Camera;
typedef SmartProjectionFactor<Camera> SmartFactor;
static Cal3_S2 K(fov, w, h);
static Cal3_S2 K2(1500, 1200, 0, w, h);
Camera level_camera(level_pose, K2);
Camera level_camera_right(pose_right, K2);
Point2 level_uv = level_camera.project(landmark1);
Point2 level_uv_right = level_camera_right.project(landmark1);
Camera cam1(level_pose, K2);
Camera cam2(pose_right, K2);
Camera cam3(pose_above, K2);
typedef GeneralSFMFactor<Camera, Point3> SFMFactor;
SmartProjectionParams params;
}

/* ************************************************************************* */
// default Cal3_S2 poses
namespace vanillaPose {
typedef PinholePose<Cal3_S2> Camera;
typedef CameraSet<Camera> Cameras;
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;
typedef SmartProjectionRigFactor<Camera> SmartRigFactor;
static Cal3_S2::shared_ptr sharedK(new Cal3_S2(fov, w, h));
Camera level_camera(level_pose, sharedK);
Camera level_camera_right(pose_right, sharedK);
Camera cam1(level_pose, sharedK);
Camera cam2(pose_right, sharedK);
Camera cam3(pose_above, sharedK);
}

/* ************************************************************************* */
// default Cal3_S2 poses
namespace vanillaPose2 {
typedef PinholePose<Cal3_S2> Camera;
typedef CameraSet<Camera> Cameras;
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;
typedef SmartProjectionRigFactor<Camera> SmartRigFactor;
static Cal3_S2::shared_ptr sharedK2(new Cal3_S2(1500, 1200, 0, 640, 480));
Camera level_camera(level_pose, sharedK2);
Camera level_camera_right(pose_right, sharedK2);
Camera cam1(level_pose, sharedK2);
Camera cam2(pose_right, sharedK2);
Camera cam3(pose_above, sharedK2);
}

/* *************************************************************************/
// Cal3Bundler cameras
namespace bundler {
typedef PinholeCamera<Cal3Bundler> Camera;
typedef CameraSet<Camera> Cameras;
typedef SmartProjectionFactor<Camera> SmartFactor;
static Cal3Bundler K(500, 1e-3, 1e-3, 0, 0);
Camera level_camera(level_pose, K);
Camera level_camera_right(pose_right, K);
Point2 level_uv = level_camera.project(landmark1);
Point2 level_uv_right = level_camera_right.project(landmark1);
Pose3 pose1 = level_pose;
Camera cam1(level_pose, K);
Camera cam2(pose_right, K);
Camera cam3(pose_above, K);
typedef GeneralSFMFactor<Camera, Point3> SFMFactor;
}
/* *************************************************************************/
// Cal3Bundler poses
namespace bundlerPose {
typedef PinholePose<Cal3Bundler> Camera;
typedef CameraSet<Camera> Cameras;
typedef SmartProjectionPoseFactor<Cal3Bundler> SmartFactor;
typedef SmartProjectionRigFactor<Camera> SmartRigFactor;
static boost::shared_ptr<Cal3Bundler> sharedBundlerK(
    new Cal3Bundler(500, 1e-3, 1e-3, 1000, 2000));
Camera level_camera(level_pose, sharedBundlerK);
Camera level_camera_right(pose_right, sharedBundlerK);
Camera cam1(level_pose, sharedBundlerK);
Camera cam2(pose_right, sharedBundlerK);
Camera cam3(pose_above, sharedBundlerK);
}
/* *************************************************************************/

template<class CAMERA>
CAMERA perturbCameraPose(const CAMERA& camera) {
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Pose3 cameraPose = camera.pose();
  Pose3 perturbedCameraPose = cameraPose.compose(noise_pose);
  return CAMERA(perturbedCameraPose, camera.calibration());
}

template<class CAMERA>
void projectToMultipleCameras(const CAMERA& cam1, const CAMERA& cam2,
    const CAMERA& cam3, Point3 landmark, typename CAMERA::MeasurementVector& measurements_cam) {
  Point2 cam1_uv1 = cam1.project(landmark);
  Point2 cam2_uv1 = cam2.project(landmark);
  Point2 cam3_uv1 = cam3.project(landmark);
  measurements_cam.push_back(cam1_uv1);
  measurements_cam.push_back(cam2_uv1);
  measurements_cam.push_back(cam3_uv1);
}

/* ************************************************************************* */

