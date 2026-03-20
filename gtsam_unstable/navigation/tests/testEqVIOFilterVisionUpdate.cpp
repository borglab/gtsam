/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file testEqVIOFilterVisionUpdate.cpp

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <memory>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace {

State MakeState1() {
  SensorState sensor;
  sensor.inputBias = Bias(Vector3(0.03, -0.01, 0.02), Vector3(0.1, -0.2, 0.05));
  sensor.pose = Pose3(Rot3::RzRyRx(0.2, -0.1, 0.15), Point3(0.4, -0.2, 1.0));
  sensor.velocity = Vector3(0.5, -0.3, 0.2);
  sensor.cameraOffset =
      Pose3(Rot3::RzRyRx(-0.08, 0.04, -0.03), Point3(0.1, 0.0, 0.05));
  return State(sensor, {{Point3(0.3, -0.15, 4.5), 10}});
}

}  // namespace

TEST(EqVIOFilter, VisionUpdate) {
  EqVIOFilterParams params;

  const State xi0 = MakeState1();
  const Matrix Sigma0 = Matrix::Identity(xi0.dim(), xi0.dim()) * 1e-3;
  EqVIOFilter filter(xi0, Sigma0, params);

  IMUInput imu;
  imu.stamp = 0.0;
  imu.gyr = Vector3::Zero();
  imu.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
  filter.propagate(imu, 0.01);

  auto camera =
      std::make_shared<CameraModel>(Pose3::Identity(), Cal3_S2(1, 1, 0, 0, 0));
  const VisionMeasurement meas = measureSystemState(filter.stateEstimate(), camera);
  filter.correct(meas, camera);

  EXPECT_LONGS_EQUAL(1, filter.stateEstimate().n());
  EXPECT(filter.view().Sigma.array().isFinite().all());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
