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
};

}  // namespace

TEST(EqVIOFilter, InitAndPropagation) {
  EqVIOFilterParams params;
  params.coordinateChoice = CoordinateChoice::InvDepth;
  params.useDiscreteVelocityLift = true;

  EqVIOFilter filter(params);
  EXPECT(!filter.isInitialized());

  IMUInput imu0;
  imu0.stamp = 1.0;
  imu0.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
  imu0.gyr = Vector3::Zero();
  filter.processIMUData(imu0);
  EXPECT(filter.isInitialized());

  IMUInput imu1 = imu0;
  imu1.stamp = 1.01;
  filter.processIMUData(imu1);

  auto camera = std::make_shared<SimplePinholeCamera>();
  VisionMeasurement meas;
  filter.processVisionData(1.02, meas, camera);

  EXPECT(std::abs(filter.currentTime() - 1.02) < 1e-12);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
