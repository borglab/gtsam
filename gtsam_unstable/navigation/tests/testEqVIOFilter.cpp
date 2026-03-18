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

#include <cmath>
#include <memory>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace {

class SimplePinholeCamera final : public VIOCameraModel {
 public:
  Point2 projectPoint(const Point3& p) const override {
    if (std::abs(p.z()) < 1e-12) {
      throw std::invalid_argument("SimplePinholeCamera: z near zero");
    }
    return Point2(p.x() / p.z(), p.y() / p.z());
  }

  Vector3 undistortPoint(const Point2& y) const override {
    return Vector3(y.x(), y.y(), 1.0).normalized();
  }

  Matrix23 projectionJacobian(const Vector3& y) const override {
    if (std::abs(y.z()) < 1e-12) {
      throw std::invalid_argument("SimplePinholeCamera: z near zero");
    }
    Matrix23 J;
    const double z2 = y.z() * y.z();
    J << 1.0 / y.z(), 0.0, -y.x() / z2, 0.0, 1.0 / y.z(), -y.y() / z2;
    return J;
  }
};

}  // namespace

TEST(EqVIOFilter, Smoke) {
  EqVIOFilterParams params;
  params.removeLostLandmarks = false;

  VIOSensorState sensor;
  sensor.inputBias = VIOBias::Identity();
  sensor.pose = Pose3::Identity();
  sensor.velocity.setZero();
  sensor.cameraOffset = Pose3::Identity();

  VIOState xi0(sensor, {{Point3(0.8, -0.2, 4.5), 11}, {Point3(-0.6, 0.3, 3.8), 22}});
  Matrix Sigma0 = Matrix::Identity(xi0.dim(), xi0.dim()) * 1e-3;

  EqVIOFilter filter(xi0, Sigma0, params, 0.0);
  auto camera = std::make_shared<SimplePinholeCamera>();

  const double dt = 0.01;
  double t = 0.0;
  for (int k = 0; k < 100; ++k) {
    t += dt;
    IMUInput imu;
    imu.stamp = t;
    imu.gyr = Vector3::Zero();
    imu.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
    filter.processIMUData(imu);

    if (k % 5 == 0) {
      VisionMeasurement y = measureSystemState(filter.stateEstimate(), camera);
      filter.processVisionData(t, y, camera);
    }
  }

  const VIOState est = filter.stateEstimate();
  EXPECT_LONGS_EQUAL(2, est.n());
  EXPECT_LONGS_EQUAL(xi0.dim(), filter.view().Sigma.rows());
  EXPECT_LONGS_EQUAL(xi0.dim(), filter.view().Sigma.cols());
  EXPECT(filter.view().Sigma.array().isFinite().all());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
