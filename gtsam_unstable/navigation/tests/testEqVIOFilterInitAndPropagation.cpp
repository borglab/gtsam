/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file testEqVIOFilterInitAndPropagation.cpp

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <memory>

using namespace gtsam;
using namespace gtsam::eqvio;

TEST(EqVIOFilter, InitAndPropagation) {
  EqVIOFilterParams params;

  EqVIOFilter filter(params);

  IMUInput imu0;
  imu0.stamp = 1.0;
  imu0.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
  imu0.gyr = Vector3::Zero();
  filter.processIMUData(imu0);

  IMUInput imu1 = imu0;
  imu1.stamp = 1.01;
  filter.processIMUData(imu1);

  auto camera =
      std::make_shared<VIOCameraModel>(Pose3::Identity(), Cal3_S2(1, 1, 0, 0, 0));
  VisionMeasurement meas;
  filter.processVisionData(1.02, meas, camera);

  EXPECT(std::abs(filter.currentTime() - 1.02) < 1e-12);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
