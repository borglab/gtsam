/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file testEqVIOFilter.cpp

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <memory>

using namespace gtsam;
using namespace gtsam::eqvio;

TEST(EqVIOFilter, Smoke) {
  EqVIOFilterParams params;

  SensorState sensor;
  sensor.inputBias = Bias::Identity();
  sensor.pose = Pose3::Identity();
  sensor.velocity.setZero();
  sensor.cameraOffset = Pose3::Identity();

  State xi0(sensor, {{Point3(0.8, -0.2, 4.5), 11}, {Point3(-0.6, 0.3, 3.8), 22}});
  Matrix Sigma0 = Matrix::Identity(xi0.dim(), xi0.dim()) * 1e-3;

  EqVIOFilter filter(xi0, Sigma0, params);
  auto camera =
      std::make_shared<CameraModel>(Pose3::Identity(), Cal3_S2(1, 1, 0, 0, 0));

  const double dt = 0.01;
  double t = 0.0;
  for (int k = 0; k < 100; ++k) {
    t += dt;
    IMUInput imu;
    imu.stamp = t;
    imu.gyr = Vector3::Zero();
    imu.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
    filter.propagate(imu, dt);

    if (k % 5 == 0) {
      VisionMeasurement y = measureSystemState(filter.stateEstimate(), camera);
      filter.correct(y, camera);
    }
  }

  const State est = filter.stateEstimate();
  EXPECT_LONGS_EQUAL(2, est.n());
  EXPECT_LONGS_EQUAL(xi0.dim(), filter.view().Sigma.rows());
  EXPECT_LONGS_EQUAL(xi0.dim(), filter.view().Sigma.cols());
  EXPECT(filter.view().Sigma.array().isFinite().all());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
