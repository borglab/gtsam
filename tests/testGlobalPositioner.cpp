/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGlobalPositioner.cpp
 * @author Kathir Gounder
 * @date April 2026
 * @brief Test global positioning: recover camera and landmark positions
 *        from camera-to-point direction measurements.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/sfm/GlobalPositioner.h>

using namespace std;
using namespace gtsam;
using symbol_shorthand::L;
using symbol_shorthand::X;

// Synthetic bipartite scene: 3 cameras looking at 4 landmarks.
// Cameras are in a triangle, landmarks are in front of them.
static const Point3 kCam0(0, 0, 0);
static const Point3 kCam1(2, 0, 0);
static const Point3 kCam2(1, -1, 0);
static const Point3 kPt0(0.5, 0.5, 5);
static const Point3 kPt1(1.5, 0.5, 5);
static const Point3 kPt2(1.0, -0.5, 5);
static const Point3 kPt3(1.0, 0.5, 8);

// Build ground-truth direction measurements from known positions.
GlobalPositioner::CameraPointDirections SimulateDirections(
    const vector<Point3>& cameras, const vector<Point3>& landmarks) {
  auto noiseModel = noiseModel::Isotropic::Sigma(2, 0.01);
  GlobalPositioner::CameraPointDirections measurements;
  for (size_t i = 0; i < cameras.size(); i++) {
    for (size_t j = 0; j < landmarks.size(); j++) {
      Unit3 direction(landmarks[j] - cameras[i]);
      measurements.emplace_back(X(i), L(j), direction, noiseModel);
    }
  }
  return measurements;
}

/* ************************************************************************* */
// Test that GlobalPositioner recovers camera and landmark positions from
// clean direction measurements in a simple bipartite scene.
TEST(GlobalPositioner, ThreeCamerasFourLandmarks) {
  vector<Point3> cameras = {kCam0, kCam1, kCam2};
  vector<Point3> landmarks = {kPt0, kPt1, kPt2, kPt3};
  auto measurements = SimulateDirections(cameras, landmarks);

  set<Key> cameraKeys = {X(0), X(1), X(2)};
  set<Key> landmarkKeys = {L(0), L(1), L(2), L(3)};

  GlobalPositioner gp;
  const auto result = gp.run(measurements, cameraKeys, landmarkKeys, X(0));

  // Anchor camera should be at origin.
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(X(0)), 1e-3));

  // Check relative positions are correct (up to global scale).
  // The scale is determined by BATA consensus, so we compare directions
  // and relative distances rather than absolute positions.
  Point3 estimated_cam1 = result.at<Point3>(X(1));
  Point3 estimated_pt0 = result.at<Point3>(L(0));

  // Camera 1 should be roughly along the +X direction from camera 0.
  Unit3 expected_dir_01(kCam1 - kCam0);
  Unit3 actual_dir_01(estimated_cam1);
  EXPECT(assert_equal(expected_dir_01, actual_dir_01, 1e-2));

  // Landmark 0 should be in front of camera 0 (positive Z).
  EXPECT(estimated_pt0(2) > 0);
}

/* ************************************************************************* */
// Test that the anchor camera validation works.
TEST(GlobalPositioner, AnchorValidation) {
  vector<Point3> cameras = {kCam0, kCam1};
  vector<Point3> landmarks = {kPt0};
  auto measurements = SimulateDirections(cameras, landmarks);

  set<Key> cameraKeys = {X(0), X(1)};
  set<Key> landmarkKeys = {L(0)};

  GlobalPositioner gp;

  // Passing a landmark key as anchor should throw.
  CHECK_EXCEPTION(gp.run(measurements, cameraKeys, landmarkKeys, L(0)),
                  invalid_argument);

  // Passing a nonexistent key as anchor should throw.
  CHECK_EXCEPTION(gp.run(measurements, cameraKeys, landmarkKeys, X(99)),
                  invalid_argument);
}

/* ************************************************************************* */
// Test initializeRandomly produces the right number of variables.
TEST(GlobalPositioner, InitializeRandomly) {
  vector<Point3> cameras = {kCam0, kCam1};
  vector<Point3> landmarks = {kPt0, kPt1};
  auto measurements = SimulateDirections(cameras, landmarks);

  set<Key> cameraKeys = {X(0), X(1)};
  set<Key> landmarkKeys = {L(0), L(1)};

  GlobalPositioner gp;
  const auto initial =
      gp.initializeRandomly(cameraKeys, landmarkKeys, measurements);

  // Should have 2 cameras + 2 landmarks + 4 scale variables = 8 entries.
  EXPECT_LONGS_EQUAL(8, initial.size());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
