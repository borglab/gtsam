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
    const double z2 = y.z() * y.z();
    Matrix23 J;
    J << 1.0 / y.z(), 0.0, -y.x() / z2, 0.0, 1.0 / y.z(), -y.y() / z2;
    return J;
  }
};

}  // namespace

TEST(EqVIOFilter, DynamicLandmarksAddRemove) {
  EqVIOFilterParams params;
  params.coordinateChoice = CoordinateChoice::InvDepth;
  params.removeLostLandmarks = true;
  params.useDiscreteVelocityLift = true;
  params.useDiscreteInnovationLift = true;
  params.initialPointDepth = 5.0;

  EqVIOFilter filter(params);
  auto camera = std::make_shared<SimplePinholeCamera>();

  IMUInput imu0;
  imu0.stamp = 0.0;
  imu0.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
  imu0.gyr = Vector3::Zero();
  filter.processIMUData(imu0);

  IMUInput imu1 = imu0;
  imu1.stamp = 0.01;
  filter.processIMUData(imu1);

  VisionMeasurement meas1;
  meas1[1] = camera->projectPoint(Point3(0.2, -0.1, 3.5));
  meas1[2] = camera->projectPoint(Point3(-0.3, 0.15, 4.0));
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
