/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file testEqVIOFilterDynamicLandmarks.cpp

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <memory>

using namespace gtsam;
using namespace gtsam::eqvio;

TEST(EqVIOFilter, DynamicLandmarksAddRemove) {
  EqVIOFilterParams params;
  params.removeLostLandmarks = true;
  params.initialPointDepth = 5.0;

  EqVIOFilter filter(params);
  auto camera =
      std::make_shared<VIOCameraModel>(Pose3::Identity(), Cal3_S2(1, 1, 0, 0, 0));

  IMUInput imu0;
  imu0.stamp = 0.0;
  imu0.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
  imu0.gyr = Vector3::Zero();
  filter.processIMUData(imu0);

  IMUInput imu1 = imu0;
  imu1.stamp = 0.01;
  filter.processIMUData(imu1);

  VisionMeasurement meas1;
  meas1[1] = camera->project2(Point3(0.2, -0.1, 3.5));
  meas1[2] = camera->project2(Point3(-0.3, 0.15, 4.0));
  filter.processVisionData(0.02, meas1, camera);
  EXPECT_LONGS_EQUAL(2, filter.stateEstimate().n());

  IMUInput imu2 = imu0;
  imu2.stamp = 0.02;
  filter.processIMUData(imu2);

  VisionMeasurement meas2;
  meas2[1] = meas1.at(1);
  filter.processVisionData(0.03, meas2, camera);

  const VIOState est = filter.stateEstimate();
  EXPECT_LONGS_EQUAL(1, est.n());
  EXPECT_LONGS_EQUAL(1, est.cameraLandmarks.front().id);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
