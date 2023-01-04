/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTriangulation.cpp
 * @brief triangulation utilities
 * @date July 30th, 2013
 * @author Chris Beall (cbeall3)
 * @author Luca Carlone
 * @author Akshay Krishnan
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/CameraSet.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/SphericalCamera.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/StereoFactor.h>

#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

// Some common constants

static const boost::shared_ptr<Cal3_S2> kSharedCal =  //
    boost::make_shared<Cal3_S2>(1500, 1200, 0.1, 640, 480);

// Looking along X-axis, 1 meter above ground plane (x-y)
static const Rot3 upright = Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2);
static const Pose3 kPose1 = Pose3(upright, gtsam::Point3(0, 0, 1));
static const PinholeCamera<Cal3_S2> kCamera1(kPose1, *kSharedCal);

// create second camera 1 meter to the right of first camera
static const Pose3 kPose2 = kPose1 * Pose3(Rot3(), Point3(1, 0, 0));
static const PinholeCamera<Cal3_S2> kCamera2(kPose2, *kSharedCal);

// landmark ~5 meters infront of camera
static const Point3 kLandmark(5, 0.5, 1.2);

// 1. Project two landmarks into two cameras and triangulate
static const Point2 kZ1 = kCamera1.project(kLandmark);
static const Point2 kZ2 = kCamera2.project(kLandmark);

//******************************************************************************
// Simple test with a well-behaved two camera situation
TEST(triangulation, twoPoses) {
  vector<Pose3> poses;
  Point2Vector measurements;

  poses += kPose1, kPose2;
  measurements += kZ1, kZ2;

  double rank_tol = 1e-9;

  // 1. Test simple DLT, perfect in no noise situation
  bool optimize = false;
  boost::optional<Point3> actual1 =  //
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements, rank_tol, optimize);
  EXPECT(assert_equal(kLandmark, *actual1, 1e-7));

  // 2. test with optimization on, same answer
  optimize = true;
  boost::optional<Point3> actual2 =  //
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements, rank_tol, optimize);
  EXPECT(assert_equal(kLandmark, *actual2, 1e-7));

  // 3. Add some noise and try again: result should be ~ (4.995,
  // 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);
  optimize = false;
  boost::optional<Point3> actual3 =  //
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements, rank_tol, optimize);
  EXPECT(assert_equal(Point3(4.995, 0.499167, 1.19814), *actual3, 1e-4));

  // 4. Now with optimization on
  optimize = true;
  boost::optional<Point3> actual4 =  //
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements, rank_tol, optimize);
  EXPECT(assert_equal(Point3(4.995, 0.499167, 1.19814), *actual4, 1e-4));
}

TEST(triangulation, twoCamerasUsingLOST) {
  CameraSet<PinholeCamera<Cal3_S2>> cameras;
  cameras.push_back(kCamera1);
  cameras.push_back(kCamera2);

  Point2Vector measurements = {kZ1, kZ2};
  SharedNoiseModel measurementNoise = noiseModel::Isotropic::Sigma(2, 1e-4);
  double rank_tol = 1e-9;

  // 1. Test simple triangulation, perfect in no noise situation
  boost::optional<Point3> actual1 =  //
      triangulatePoint3<PinholeCamera<Cal3_S2>>(cameras, measurements, rank_tol,
                                                /*optimize=*/false,
                                                measurementNoise,
                                                /*useLOST=*/true);
  EXPECT(assert_equal(kLandmark, *actual1, 1e-12));

  // 3. Add some noise and try again: result should be ~ (4.995,
  // 0.499167, 1.19814)
  measurements[0] += Point2(0.1, 0.5);
  measurements[1] += Point2(-0.2, 0.3);
  boost::optional<Point3> actual2 =  //
      triangulatePoint3<PinholeCamera<Cal3_S2>>(
          cameras, measurements, rank_tol,
          /*optimize=*/false, measurementNoise,
          /*use_lost_triangulation=*/true);
  EXPECT(assert_equal(Point3(4.995, 0.499167, 1.19814), *actual2, 1e-4));
}

TEST(triangulation, twoCamerasLOSTvsDLT) {
  // LOST has been shown to preform better when the point is much closer to one
  // camera compared to another. This unit test checks this configuration.
  const Cal3_S2 identityK;
  const Pose3 pose1;
  const Pose3 pose2(Rot3(), Point3(5., 0., -5.));
  CameraSet<PinholeCamera<Cal3_S2>> cameras;
  cameras.emplace_back(pose1, identityK);
  cameras.emplace_back(pose2, identityK);

  Point3 landmark(0, 0, 1);
  Point2 x1_noisy = cameras[0].project(landmark) + Point2(0.00817, 0.00977);
  Point2 x2_noisy = cameras[1].project(landmark) + Point2(-0.00610, 0.01969);
  Point2Vector measurements = {x1_noisy, x2_noisy};

  const double rank_tol = 1e-9;
  SharedNoiseModel measurementNoise = noiseModel::Isotropic::Sigma(2, 1e-2);

  boost::optional<Point3> estimateLOST =  //
      triangulatePoint3<PinholeCamera<Cal3_S2>>(cameras, measurements, rank_tol,
                                                /*optimize=*/false,
                                                measurementNoise,
                                                /*useLOST=*/true);

  // These values are from a MATLAB implementation.
  EXPECT(assert_equal(Point3(0.007, 0.011, 0.945), *estimateLOST, 1e-3));

  boost::optional<Point3> estimateDLT =
      triangulatePoint3<Cal3_S2>(cameras, measurements, rank_tol, false);

  // The LOST estimate should have a smaller error.
  EXPECT((landmark - *estimateLOST).norm() <= (landmark - *estimateDLT).norm());
}

//******************************************************************************
// Simple test with a well-behaved two camera situation with Cal3DS2 calibration.
TEST(triangulation, twoPosesCal3DS2) {
  static const boost::shared_ptr<Cal3DS2> sharedDistortedCal =  //
      boost::make_shared<Cal3DS2>(1500, 1200, 0, 640, 480, -.3, 0.1, 0.0001,
                                  -0.0003);

  PinholeCamera<Cal3DS2> camera1Distorted(kPose1, *sharedDistortedCal);

  PinholeCamera<Cal3DS2> camera2Distorted(kPose2, *sharedDistortedCal);

  // 0. Project two landmarks into two cameras and triangulate
  Point2 z1Distorted = camera1Distorted.project(kLandmark);
  Point2 z2Distorted = camera2Distorted.project(kLandmark);

  vector<Pose3> poses;
  Point2Vector measurements;

  poses += kPose1, kPose2;
  measurements += z1Distorted, z2Distorted;

  double rank_tol = 1e-9;

  // 1. Test simple DLT, perfect in no noise situation
  bool optimize = false;
  boost::optional<Point3> actual1 =  //
      triangulatePoint3<Cal3DS2>(poses, sharedDistortedCal, measurements,
                                 rank_tol, optimize);
  EXPECT(assert_equal(kLandmark, *actual1, 1e-7));

  // 2. test with optimization on, same answer
  optimize = true;
  boost::optional<Point3> actual2 =  //
      triangulatePoint3<Cal3DS2>(poses, sharedDistortedCal, measurements,
                                 rank_tol, optimize);
  EXPECT(assert_equal(kLandmark, *actual2, 1e-7));

  // 3. Add some noise and try again: result should be ~ (4.995,
  // 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);
  optimize = false;
  boost::optional<Point3> actual3 =  //
      triangulatePoint3<Cal3DS2>(poses, sharedDistortedCal, measurements,
                                 rank_tol, optimize);
  EXPECT(assert_equal(Point3(4.995, 0.499167, 1.19814), *actual3, 1e-3));

  // 4. Now with optimization on
  optimize = true;
  boost::optional<Point3> actual4 =  //
      triangulatePoint3<Cal3DS2>(poses, sharedDistortedCal, measurements,
                                 rank_tol, optimize);
  EXPECT(assert_equal(Point3(4.995, 0.499167, 1.19814), *actual4, 1e-3));
}

//******************************************************************************
// Simple test with a well-behaved two camera situation with Fisheye
// calibration.
TEST(triangulation, twoPosesFisheye) {
  using Calibration = Cal3Fisheye;
  static const boost::shared_ptr<Calibration> sharedDistortedCal =  //
      boost::make_shared<Calibration>(1500, 1200, .1, 640, 480, -.3, 0.1,
                                      0.0001, -0.0003);

  PinholeCamera<Calibration> camera1Distorted(kPose1, *sharedDistortedCal);

  PinholeCamera<Calibration> camera2Distorted(kPose2, *sharedDistortedCal);

  // 0. Project two landmarks into two cameras and triangulate
  Point2 z1Distorted = camera1Distorted.project(kLandmark);
  Point2 z2Distorted = camera2Distorted.project(kLandmark);

  vector<Pose3> poses;
  Point2Vector measurements;

  poses += kPose1, kPose2;
  measurements += z1Distorted, z2Distorted;

  double rank_tol = 1e-9;

  // 1. Test simple DLT, perfect in no noise situation
  bool optimize = false;
  boost::optional<Point3> actual1 =  //
      triangulatePoint3<Calibration>(poses, sharedDistortedCal, measurements,
                                     rank_tol, optimize);
  EXPECT(assert_equal(kLandmark, *actual1, 1e-7));

  // 2. test with optimization on, same answer
  optimize = true;
  boost::optional<Point3> actual2 =  //
      triangulatePoint3<Calibration>(poses, sharedDistortedCal, measurements,
                                     rank_tol, optimize);
  EXPECT(assert_equal(kLandmark, *actual2, 1e-7));

  // 3. Add some noise and try again: result should be ~ (4.995,
  // 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);
  optimize = false;
  boost::optional<Point3> actual3 =  //
      triangulatePoint3<Calibration>(poses, sharedDistortedCal, measurements,
                                     rank_tol, optimize);
  EXPECT(assert_equal(Point3(4.995, 0.499167, 1.19814), *actual3, 1e-3));

  // 4. Now with optimization on
  optimize = true;
  boost::optional<Point3> actual4 =  //
      triangulatePoint3<Calibration>(poses, sharedDistortedCal, measurements,
                                     rank_tol, optimize);
  EXPECT(assert_equal(Point3(4.995, 0.499167, 1.19814), *actual4, 1e-3));
}

//******************************************************************************
// Similar, but now with Bundler calibration
TEST(triangulation, twoPosesBundler) {
  boost::shared_ptr<Cal3Bundler> bundlerCal =  //
      boost::make_shared<Cal3Bundler>(1500, 0.1, 0.2, 640, 480);
  PinholeCamera<Cal3Bundler> camera1(kPose1, *bundlerCal);
  PinholeCamera<Cal3Bundler> camera2(kPose2, *bundlerCal);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 z1 = camera1.project(kLandmark);
  Point2 z2 = camera2.project(kLandmark);

  vector<Pose3> poses;
  Point2Vector measurements;

  poses += kPose1, kPose2;
  measurements += z1, z2;

  bool optimize = true;
  double rank_tol = 1e-9;

  boost::optional<Point3> actual =  //
      triangulatePoint3<Cal3Bundler>(poses, bundlerCal, measurements, rank_tol,
                                     optimize);
  EXPECT(assert_equal(kLandmark, *actual, 1e-7));

  // Add some noise and try again
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> actual2 =  //
      triangulatePoint3<Cal3Bundler>(poses, bundlerCal, measurements, rank_tol,
                                     optimize);
  EXPECT(assert_equal(Point3(4.995, 0.499167, 1.19847), *actual2, 1e-3));
}

//******************************************************************************
TEST(triangulation, fourPoses) {
  vector<Pose3> poses;
  Point2Vector measurements;

  poses += kPose1, kPose2;
  measurements += kZ1, kZ2;

  boost::optional<Point3> actual =
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements);
  EXPECT(assert_equal(kLandmark, *actual, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995,
  // 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> actual2 =  //
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements);
  EXPECT(assert_equal(kLandmark, *actual2, 1e-2));

  // 3. Add a slightly rotated third camera above, again with measurement noise
  Pose3 pose3 = kPose1 * Pose3(Rot3::Ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  PinholeCamera<Cal3_S2> camera3(pose3, *kSharedCal);
  Point2 z3 = camera3.project(kLandmark);

  poses += pose3;
  measurements += z3 + Point2(0.1, -0.1);

  boost::optional<Point3> triangulated_3cameras =  //
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements);
  EXPECT(assert_equal(kLandmark, *triangulated_3cameras, 1e-2));

  // Again with nonlinear optimization
  boost::optional<Point3> triangulated_3cameras_opt =
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements, 1e-9, true);
  EXPECT(assert_equal(kLandmark, *triangulated_3cameras_opt, 1e-2));

  // 4. Test failure: Add a 4th camera facing the wrong way
  Pose3 pose4 = Pose3(Rot3::Ypr(M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  PinholeCamera<Cal3_S2> camera4(pose4, *kSharedCal);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  CHECK_EXCEPTION(camera4.project(kLandmark), CheiralityException);

  poses += pose4;
  measurements += Point2(400, 400);

  CHECK_EXCEPTION(triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements),
                  TriangulationCheiralityException);
#endif
}

//******************************************************************************
TEST(triangulation, threePoses_robustNoiseModel) {

  Pose3 pose3 = kPose1 * Pose3(Rot3::Ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  PinholeCamera<Cal3_S2> camera3(pose3, *kSharedCal);
  Point2 z3 = camera3.project(kLandmark);

  vector<Pose3> poses;
  Point2Vector measurements;
  poses += kPose1, kPose2, pose3;
  measurements += kZ1, kZ2, z3;

  // noise free, so should give exactly the landmark
  boost::optional<Point3> actual =
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements);
  EXPECT(assert_equal(kLandmark, *actual, 1e-2));

  // Add outlier
  measurements.at(0) += Point2(100, 120); // very large pixel noise!

  // now estimate does not match landmark
  boost::optional<Point3> actual2 =  //
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements);
  // DLT is surprisingly robust, but still off (actual error is around 0.26m):
  EXPECT( (kLandmark - *actual2).norm() >= 0.2);
  EXPECT( (kLandmark - *actual2).norm() <= 0.5);

  // Again with nonlinear optimization
  boost::optional<Point3> actual3 =
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements, 1e-9, true);
  // result from nonlinear (but non-robust optimization) is close to DLT and still off
  EXPECT(assert_equal(*actual2, *actual3, 0.1));

  // Again with nonlinear optimization, this time with robust loss
  auto model = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345), noiseModel::Unit::Create(2));
  boost::optional<Point3> actual4 = triangulatePoint3<Cal3_S2>(
      poses, kSharedCal, measurements, 1e-9, true, model);
  // using the Huber loss we now have a quite small error!! nice!
  EXPECT(assert_equal(kLandmark, *actual4, 0.05));
}

//******************************************************************************
TEST(triangulation, fourPoses_robustNoiseModel) {

  Pose3 pose3 = kPose1 * Pose3(Rot3::Ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  PinholeCamera<Cal3_S2> camera3(pose3, *kSharedCal);
  Point2 z3 = camera3.project(kLandmark);

  vector<Pose3> poses;
  Point2Vector measurements;
  poses += kPose1, kPose1, kPose2, pose3; // 2 measurements from pose 1
  measurements += kZ1, kZ1, kZ2, z3;

  // noise free, so should give exactly the landmark
  boost::optional<Point3> actual =
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements);
  EXPECT(assert_equal(kLandmark, *actual, 1e-2));

  // Add outlier
  measurements.at(0) += Point2(100, 120); // very large pixel noise!
  // add noise on other measurements:
  measurements.at(1) += Point2(0.1, 0.2); // small noise
  measurements.at(2) += Point2(0.2, 0.2);
  measurements.at(3) += Point2(0.3, 0.1);

  // now estimate does not match landmark
  boost::optional<Point3> actual2 =  //
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements);
  // DLT is surprisingly robust, but still off (actual error is around 0.17m):
  EXPECT( (kLandmark - *actual2).norm() >= 0.1);
  EXPECT( (kLandmark - *actual2).norm() <= 0.5);

  // Again with nonlinear optimization
  boost::optional<Point3> actual3 =
      triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements, 1e-9, true);
  // result from nonlinear (but non-robust optimization) is close to DLT and still off
  EXPECT(assert_equal(*actual2, *actual3, 0.1));

  // Again with nonlinear optimization, this time with robust loss
  auto model = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345), noiseModel::Unit::Create(2));
  boost::optional<Point3> actual4 = triangulatePoint3<Cal3_S2>(
      poses, kSharedCal, measurements, 1e-9, true, model);
  // using the Huber loss we now have a quite small error!! nice!
  EXPECT(assert_equal(kLandmark, *actual4, 0.05));
}

//******************************************************************************
TEST(triangulation, fourPoses_distinct_Ks) {
  Cal3_S2 K1(1500, 1200, 0, 640, 480);
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  PinholeCamera<Cal3_S2> camera1(kPose1, K1);

  // create second camera 1 meter to the right of first camera
  Cal3_S2 K2(1600, 1300, 0, 650, 440);
  PinholeCamera<Cal3_S2> camera2(kPose2, K2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 z1 = camera1.project(kLandmark);
  Point2 z2 = camera2.project(kLandmark);

  CameraSet<PinholeCamera<Cal3_S2>> cameras;
  Point2Vector measurements;

  cameras += camera1, camera2;
  measurements += z1, z2;

  boost::optional<Point3> actual =  //
      triangulatePoint3<Cal3_S2>(cameras, measurements);
  EXPECT(assert_equal(kLandmark, *actual, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995,
  // 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> actual2 =  //
      triangulatePoint3<Cal3_S2>(cameras, measurements);
  EXPECT(assert_equal(kLandmark, *actual2, 1e-2));

  // 3. Add a slightly rotated third camera above, again with measurement noise
  Pose3 pose3 = kPose1 * Pose3(Rot3::Ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  Cal3_S2 K3(700, 500, 0, 640, 480);
  PinholeCamera<Cal3_S2> camera3(pose3, K3);
  Point2 z3 = camera3.project(kLandmark);

  cameras += camera3;
  measurements += z3 + Point2(0.1, -0.1);

  boost::optional<Point3> triangulated_3cameras =  //
      triangulatePoint3<Cal3_S2>(cameras, measurements);
  EXPECT(assert_equal(kLandmark, *triangulated_3cameras, 1e-2));

  // Again with nonlinear optimization
  boost::optional<Point3> triangulated_3cameras_opt =
      triangulatePoint3<Cal3_S2>(cameras, measurements, 1e-9, true);
  EXPECT(assert_equal(kLandmark, *triangulated_3cameras_opt, 1e-2));

  // 4. Test failure: Add a 4th camera facing the wrong way
  Pose3 pose4 = Pose3(Rot3::Ypr(M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  Cal3_S2 K4(700, 500, 0, 640, 480);
  PinholeCamera<Cal3_S2> camera4(pose4, K4);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  CHECK_EXCEPTION(camera4.project(kLandmark), CheiralityException);

  cameras += camera4;
  measurements += Point2(400, 400);
  CHECK_EXCEPTION(triangulatePoint3<Cal3_S2>(cameras, measurements),
                  TriangulationCheiralityException);
#endif
}

//******************************************************************************
TEST(triangulation, fourPoses_distinct_Ks_distortion) {
  Cal3DS2 K1(1500, 1200, 0, 640, 480,  -.3, 0.1, 0.0001, -0.0003);
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  PinholeCamera<Cal3DS2> camera1(kPose1, K1);

  // create second camera 1 meter to the right of first camera
  Cal3DS2 K2(1600, 1300, 0, 650, 440,  -.2, 0.05, 0.0002, -0.0001);
  PinholeCamera<Cal3DS2> camera2(kPose2, K2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 z1 = camera1.project(kLandmark);
  Point2 z2 = camera2.project(kLandmark);

  CameraSet<PinholeCamera<Cal3DS2>> cameras;
  Point2Vector measurements;

  cameras += camera1, camera2;
  measurements += z1, z2;

  boost::optional<Point3> actual =  //
          triangulatePoint3<Cal3DS2>(cameras, measurements);
  EXPECT(assert_equal(kLandmark, *actual, 1e-2));
}

//******************************************************************************
TEST(triangulation, outliersAndFarLandmarks) {
  Cal3_S2 K1(1500, 1200, 0, 640, 480);
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  PinholeCamera<Cal3_S2> camera1(kPose1, K1);

  // create second camera 1 meter to the right of first camera
  Cal3_S2 K2(1600, 1300, 0, 650, 440);
  PinholeCamera<Cal3_S2> camera2(kPose2, K2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 z1 = camera1.project(kLandmark);
  Point2 z2 = camera2.project(kLandmark);

  CameraSet<PinholeCamera<Cal3_S2>> cameras;
  Point2Vector measurements;

  cameras += camera1, camera2;
  measurements += z1, z2;

  double landmarkDistanceThreshold = 10;  // landmark is closer than that
  TriangulationParameters params(
      1.0, false, landmarkDistanceThreshold);  // all default except
                                               // landmarkDistanceThreshold
  TriangulationResult actual = triangulateSafe(cameras, measurements, params);
  EXPECT(assert_equal(kLandmark, *actual, 1e-2));
  EXPECT(actual.valid());

  landmarkDistanceThreshold = 4;  // landmark is farther than that
  TriangulationParameters params2(
      1.0, false, landmarkDistanceThreshold);  // all default except
                                               // landmarkDistanceThreshold
  actual = triangulateSafe(cameras, measurements, params2);
  EXPECT(actual.farPoint());

  // 3. Add a slightly rotated third camera above with a wrong measurement
  // (OUTLIER)
  Pose3 pose3 = kPose1 * Pose3(Rot3::Ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  Cal3_S2 K3(700, 500, 0, 640, 480);
  PinholeCamera<Cal3_S2> camera3(pose3, K3);
  Point2 z3 = camera3.project(kLandmark);

  cameras += camera3;
  measurements += z3 + Point2(10, -10);

  landmarkDistanceThreshold = 10;  // landmark is closer than that
  double outlierThreshold = 100;   // loose, the outlier is going to pass
  TriangulationParameters params3(1.0, false, landmarkDistanceThreshold,
                                  outlierThreshold);
  actual = triangulateSafe(cameras, measurements, params3);
  EXPECT(actual.valid());

  // now set stricter threshold for outlier rejection
  outlierThreshold = 5;  // tighter, the outlier is not going to pass
  TriangulationParameters params4(1.0, false, landmarkDistanceThreshold,
                                  outlierThreshold);
  actual = triangulateSafe(cameras, measurements, params4);
  EXPECT(actual.outlier());
}

//******************************************************************************
TEST(triangulation, twoIdenticalPoses) {
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  PinholeCamera<Cal3_S2> camera1(kPose1, *kSharedCal);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 z1 = camera1.project(kLandmark);

  vector<Pose3> poses;
  Point2Vector measurements;

  poses += kPose1, kPose1;
  measurements += z1, z1;

  CHECK_EXCEPTION(triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements),
                  TriangulationUnderconstrainedException);
}

//******************************************************************************
TEST(triangulation, onePose) {
  // we expect this test to fail with a TriangulationUnderconstrainedException
  // because there's only one camera observation

  vector<Pose3> poses;
  Point2Vector measurements;

  poses += Pose3();
  measurements += Point2(0, 0);

  CHECK_EXCEPTION(triangulatePoint3<Cal3_S2>(poses, kSharedCal, measurements),
                  TriangulationUnderconstrainedException);
}

//******************************************************************************
TEST(triangulation, StereotriangulateNonlinear) {
  auto stereoK = boost::make_shared<Cal3_S2Stereo>(1733.75, 1733.75, 0, 689.645,
                                                   508.835, 0.0699612);

  // two camera poses m1, m2
  Matrix4 m1, m2;
  m1 << 0.796888717, 0.603404026, -0.0295271487, 46.6673779, 0.592783835,
      -0.77156583, 0.230856632, 66.2186159, 0.116517574, -0.201470143,
      -0.9725393, -4.28382528, 0, 0, 0, 1;

  m2 << -0.955959025, -0.29288915, -0.0189328569, 45.7169799, -0.29277519,
      0.947083213, 0.131587097, 65.843136, -0.0206094928, 0.131334858,
      -0.991123524, -4.3525033, 0, 0, 0, 1;

  typedef CameraSet<StereoCamera> Cameras;
  Cameras cameras;
  cameras.push_back(StereoCamera(Pose3(m1), stereoK));
  cameras.push_back(StereoCamera(Pose3(m2), stereoK));

  StereoPoint2Vector measurements;
  measurements += StereoPoint2(226.936, 175.212, 424.469);
  measurements += StereoPoint2(339.571, 285.547, 669.973);

  Point3 initial =
      Point3(46.0536958, 66.4621179, -6.56285929);  // error: 96.5715555191

  Point3 actual = triangulateNonlinear<StereoCamera>(cameras, measurements, initial);

  Point3 expected(46.0484569, 66.4710686,
                  -6.55046613);  // error: 0.763510644187

  EXPECT(assert_equal(expected, actual, 1e-4));

  // regular stereo factor comparison - expect very similar result as above
  {
    typedef GenericStereoFactor<Pose3, Point3> StereoFactor;

    Values values;
    values.insert(Symbol('x', 1), Pose3(m1));
    values.insert(Symbol('x', 2), Pose3(m2));
    values.insert(Symbol('l', 1), initial);

    NonlinearFactorGraph graph;
    static SharedNoiseModel unit(noiseModel::Unit::Create(3));
    graph.emplace_shared<StereoFactor>(measurements[0], unit, Symbol('x', 1),
                                       Symbol('l', 1), stereoK);
    graph.emplace_shared<StereoFactor>(measurements[1], unit, Symbol('x', 2),
                                       Symbol('l', 1), stereoK);

    const SharedDiagonal posePrior = noiseModel::Isotropic::Sigma(6, 1e-9);
    graph.addPrior(Symbol('x', 1), Pose3(m1), posePrior);
    graph.addPrior(Symbol('x', 2), Pose3(m2), posePrior);

    LevenbergMarquardtOptimizer optimizer(graph, values);
    Values result = optimizer.optimize();

    EXPECT(assert_equal(expected, result.at<Point3>(Symbol('l', 1)), 1e-4));
  }

  // use Triangulation Factor directly - expect same result as above
  {
    Values values;
    values.insert(Symbol('l', 1), initial);

    NonlinearFactorGraph graph;
    static SharedNoiseModel unit(noiseModel::Unit::Create(3));

    graph.emplace_shared<TriangulationFactor<StereoCamera>>(
        cameras[0], measurements[0], unit, Symbol('l', 1));
    graph.emplace_shared<TriangulationFactor<StereoCamera>>(
        cameras[1], measurements[1], unit, Symbol('l', 1));

    LevenbergMarquardtOptimizer optimizer(graph, values);
    Values result = optimizer.optimize();

    EXPECT(assert_equal(expected, result.at<Point3>(Symbol('l', 1)), 1e-4));
  }

  // use ExpressionFactor - expect same result as above
  {
    Values values;
    values.insert(Symbol('l', 1), initial);

    NonlinearFactorGraph graph;
    static SharedNoiseModel unit(noiseModel::Unit::Create(3));

    Expression<Point3> point_(Symbol('l', 1));
    Expression<StereoCamera> camera0_(cameras[0]);
    Expression<StereoCamera> camera1_(cameras[1]);
    Expression<StereoPoint2> project0_(camera0_, &StereoCamera::project2,
                                       point_);
    Expression<StereoPoint2> project1_(camera1_, &StereoCamera::project2,
                                       point_);

    graph.addExpressionFactor(unit, measurements[0], project0_);
    graph.addExpressionFactor(unit, measurements[1], project1_);

    LevenbergMarquardtOptimizer optimizer(graph, values);
    Values result = optimizer.optimize();

    EXPECT(assert_equal(expected, result.at<Point3>(Symbol('l', 1)), 1e-4));
  }
}

//******************************************************************************
// Simple test with a well-behaved two camera situation
TEST(triangulation, twoPoses_sphericalCamera) {
  vector<Pose3> poses;
  std::vector<Unit3> measurements;

  // Project landmark into two cameras and triangulate
  SphericalCamera cam1(kPose1);
  SphericalCamera cam2(kPose2);
  Unit3 u1 = cam1.project(kLandmark);
  Unit3 u2 = cam2.project(kLandmark);

  poses += kPose1, kPose2;
  measurements += u1, u2;

  CameraSet<SphericalCamera> cameras;
  cameras.push_back(cam1);
  cameras.push_back(cam2);

  double rank_tol = 1e-9;

  // 1. Test linear triangulation via DLT
  auto projection_matrices = projectionMatricesFromCameras(cameras);
  Point3 point = triangulateDLT(projection_matrices, measurements, rank_tol);
  EXPECT(assert_equal(kLandmark, point, 1e-7));

  // 2. Test nonlinear triangulation
  point = triangulateNonlinear<SphericalCamera>(cameras, measurements, point);
  EXPECT(assert_equal(kLandmark, point, 1e-7));

  // 3. Test simple DLT, now within triangulatePoint3
  bool optimize = false;
  boost::optional<Point3> actual1 =  //
      triangulatePoint3<SphericalCamera>(cameras, measurements, rank_tol,
                                         optimize);
  EXPECT(assert_equal(kLandmark, *actual1, 1e-7));

  // 4. test with optimization on, same answer
  optimize = true;
  boost::optional<Point3> actual2 =  //
      triangulatePoint3<SphericalCamera>(cameras, measurements, rank_tol,
                                         optimize);
  EXPECT(assert_equal(kLandmark, *actual2, 1e-7));

  // 5. Add some noise and try again: result should be ~ (4.995,
  // 0.499167, 1.19814)
  measurements.at(0) =
      u1.retract(Vector2(0.01, 0.05));  // note: perturbation smaller for Unit3
  measurements.at(1) = u2.retract(Vector2(-0.02, 0.03));
  optimize = false;
  boost::optional<Point3> actual3 =  //
      triangulatePoint3<SphericalCamera>(cameras, measurements, rank_tol,
                                         optimize);
  EXPECT(assert_equal(Point3(5.9432, 0.654319, 1.48192), *actual3, 1e-3));

  // 6. Now with optimization on
  optimize = true;
  boost::optional<Point3> actual4 =  //
      triangulatePoint3<SphericalCamera>(cameras, measurements, rank_tol,
                                         optimize);
  EXPECT(assert_equal(Point3(5.9432, 0.654334, 1.48192), *actual4, 1e-3));
}

//******************************************************************************
TEST(triangulation, twoPoses_sphericalCamera_extremeFOV) {
  vector<Pose3> poses;
  std::vector<Unit3> measurements;

  // Project landmark into two cameras and triangulate
  Pose3 poseA = Pose3(
      Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
      Point3(0.0, 0.0, 0.0));  // with z pointing along x axis of global frame
  Pose3 poseB = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
                      Point3(2.0, 0.0, 0.0));  // 2m in front of poseA
  Point3 landmarkL(
      1.0, -1.0,
      0.0);  // 1m to the right of both cameras, in front of poseA, behind poseB
  SphericalCamera cam1(poseA);
  SphericalCamera cam2(poseB);
  Unit3 u1 = cam1.project(landmarkL);
  Unit3 u2 = cam2.project(landmarkL);

  EXPECT(assert_equal(Unit3(Point3(1.0, 0.0, 1.0)), u1,
                      1e-7));  // in front and to the right of PoseA
  EXPECT(assert_equal(Unit3(Point3(1.0, 0.0, -1.0)), u2,
                      1e-7));  // behind and to the right of PoseB

  poses += kPose1, kPose2;
  measurements += u1, u2;

  CameraSet<SphericalCamera> cameras;
  cameras.push_back(cam1);
  cameras.push_back(cam2);

  double rank_tol = 1e-9;

  {
    // 1. Test simple DLT, when 1 point is behind spherical camera
    bool optimize = false;
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
    CHECK_EXCEPTION(triangulatePoint3<SphericalCamera>(cameras, measurements,
                                                       rank_tol, optimize),
                    TriangulationCheiralityException);
#else  // otherwise project should not throw the exception
    boost::optional<Point3> actual1 =  //
        triangulatePoint3<SphericalCamera>(cameras, measurements, rank_tol,
                                           optimize);
    EXPECT(assert_equal(landmarkL, *actual1, 1e-7));
#endif
  }
  {
    // 2. test with optimization on, same answer
    bool optimize = true;
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
    CHECK_EXCEPTION(triangulatePoint3<SphericalCamera>(cameras, measurements,
                                                       rank_tol, optimize),
                    TriangulationCheiralityException);
#else  // otherwise project should not throw the exception
    boost::optional<Point3> actual1 =  //
        triangulatePoint3<SphericalCamera>(cameras, measurements, rank_tol,
                                           optimize);
    EXPECT(assert_equal(landmarkL, *actual1, 1e-7));
#endif
  }
}

//******************************************************************************
TEST(triangulation, reprojectionError_cameraComparison) {
  Pose3 poseA = Pose3(
      Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
      Point3(0.0, 0.0, 0.0));  // with z pointing along x axis of global frame
  Point3 landmarkL(5.0, 0.0, 0.0);  // 1m in front of poseA
  SphericalCamera sphericalCamera(poseA);
  Unit3 u = sphericalCamera.project(landmarkL);

  static Cal3_S2::shared_ptr sharedK(new Cal3_S2(60, 640, 480));
  PinholePose<Cal3_S2> pinholeCamera(poseA, sharedK);
  Vector2 px = pinholeCamera.project(landmarkL);

  // add perturbation and compare error in both cameras
  Vector2 px_noise(1.0, 2.0);  // px perturbation vertically and horizontally
  Vector2 measured_px = px + px_noise;
  Vector2 measured_px_calibrated = sharedK->calibrate(measured_px);
  Unit3 measured_u =
      Unit3(measured_px_calibrated[0], measured_px_calibrated[1], 1.0);
  Unit3 expected_measured_u =
      Unit3(px_noise[0] / sharedK->fx(), px_noise[1] / sharedK->fy(), 1.0);
  EXPECT(assert_equal(expected_measured_u, measured_u, 1e-7));

  Vector2 actualErrorPinhole =
      pinholeCamera.reprojectionError(landmarkL, measured_px);
  Vector2 expectedErrorPinhole = Vector2(-px_noise[0], -px_noise[1]);
  EXPECT(assert_equal(expectedErrorPinhole, actualErrorPinhole,
                      1e-7));  //- sign due to definition of error

  Vector2 actualErrorSpherical =
      sphericalCamera.reprojectionError(landmarkL, measured_u);
  // expectedError: not easy to calculate, since it involves the unit3 basis
  Vector2 expectedErrorSpherical(-0.00360842, 0.00180419);
  EXPECT(assert_equal(expectedErrorSpherical, actualErrorSpherical, 1e-7));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
