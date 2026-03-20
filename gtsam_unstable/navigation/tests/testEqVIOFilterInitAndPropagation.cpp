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
  EXPECT(!filter.isInitialized());

  IMUInput imu0;
  imu0.stamp = 1.0;
  imu0.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
  imu0.gyr = Vector3::Zero();
  filter.initializeFromIMU(imu0);
  EXPECT(filter.isInitialized());

  IMUInput imu1 = imu0;
  imu1.stamp = 1.01;
  filter.propagate(imu1, 0.01);
  filter.propagate(imu1, 0.01);

  auto camera =
      std::make_shared<CameraModel>(Pose3::Identity(), Cal3_S2(1, 1, 0, 0, 0));
  VisionMeasurement meas;
  filter.correct(meas, camera);

  EXPECT(filter.view().Sigma.array().isFinite().all());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
