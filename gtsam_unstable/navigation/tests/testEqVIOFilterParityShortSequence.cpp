/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file testEqVIOFilterParityShortSequence.cpp

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

TEST(EqVIOFilter, ParityShortSequence) {
  EqVIOFilterParams params;
  params.removeLostLandmarks = false;

  EqVIOFilter filter(params);
  auto camera = std::make_shared<SimplePinholeCamera>();

  IMUInput initImu;
  initImu.stamp = 0.0;
  initImu.gyr = Vector3::Zero();
  initImu.acc = Vector3(0.0, 0.0, GRAVITY_CONSTANT);
  filter.processIMUData(initImu);

  VIOState manual = filter.stateEstimate();
  double t = 0.01;
  const double dt = 0.01;
  for (int k = 0; k < 8; ++k) {
    IMUInput imu;
    imu.stamp = t;
    imu.gyr = Vector3(0.01, -0.02, 0.015);
    imu.acc = Vector3(0.02, -0.01, GRAVITY_CONSTANT - 0.03);
    imu.gyrBiasVel = Vector3(0.0005, -0.0002, 0.0001);
    imu.accBiasVel = Vector3(-0.0004, 0.0003, -0.0001);
    filter.processIMUData(imu);

    VisionMeasurement emptyMeas;
    filter.processVisionData(t + dt, emptyMeas, camera);

    manual = integrateSystemFunction(manual, imu, dt);
    t += dt;
  }

  const VIOState est = filter.stateEstimate();
  const Vector eps = manual.localCoordinates(est);
  EXPECT(eps.norm() < 2e-5);
  EXPECT(std::abs(filter.currentTime() - 0.09) < 1e-12);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
