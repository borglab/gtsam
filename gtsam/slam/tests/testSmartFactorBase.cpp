/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testSmartFactorBase.cpp
 *  @brief  Unit tests for testSmartFactorBase Class
 *  @author Frank Dellaert
 *  @date   Feb 2015
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/slam/SmartFactorBase.h>

#include "PinholeFactor.h"

namespace {
using namespace gtsam;
static SharedNoiseModel unit2(noiseModel::Unit::Create(2));
static SharedNoiseModel unit3(noiseModel::Unit::Create(3));
}  // namespace

using namespace std;

namespace gtsam {

TEST(SmartFactorBase, Pinhole) {
  PinholeFactor f = PinholeFactor(unit2);
  f.add(Point2(0, 0), 1);
  f.add(Point2(0, 0), 2);
  EXPECT_LONGS_EQUAL(2 * 2, f.dim());
}

TEST(SmartFactorBase, PinholeWithSensor) {
  Pose3 body_P_sensor(Rot3(), Point3(1, 0, 0));
  PinholeFactor f = PinholeFactor(unit2, body_P_sensor);
  EXPECT(assert_equal<Pose3>(f.body_P_sensor(), body_P_sensor));

  PinholeFactor::Cameras cameras;
  // Assume body at origin.
  Pose3 world_P_body = Pose3();
  // Camera coordinates in world frame.
  Pose3 wTc = world_P_body * body_P_sensor;
  cameras.push_back(PinholeCamera<Cal3Bundler>(wTc));

  // Simple point to project slightly off image center
  Point3 p(0, 0, 10);
  Point2 measurement = cameras[0].project(p);
  f.add(measurement, 1);

  PinholeFactor::Cameras::FBlocks Fs;
  Matrix E;
  Vector error = f.unwhitenedError<Point3>(cameras, p, Fs, E);

  Vector expectedError = Vector::Zero(2);
  Matrix29 expectedFs;
  expectedFs << -0.001, -1.00001, 0, -0.1, 0, -0.01, 0, 0, 0, 1, 0, 0, 0, -0.1,
      0, 0, 0, 0;
  Matrix23 expectedE;
  expectedE << 0.1, 0, 0.01, 0, 0.1, 0;

  EXPECT(assert_equal(error, expectedError));
  // We only have the jacobian for the 1 camera
  // Use of a lower tolerance value due to compiler precision mismatch.
  EXPECT(assert_equal(expectedFs, Fs[0], 1e-3));
  EXPECT(assert_equal(expectedE, E));
}

class StereoFactor : public SmartFactorBase<StereoCamera> {
 public:
  typedef SmartFactorBase<StereoCamera> Base;
  StereoFactor() {}
  StereoFactor(const SharedNoiseModel& sharedNoiseModel)
      : Base(sharedNoiseModel) {}
  double error(const Values& values) const override { return 0.0; }
  boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    return boost::shared_ptr<GaussianFactor>(new JacobianFactor());
  }
};

/// traits
template <>
struct traits<StereoFactor> : public Testable<StereoFactor> {};

TEST(SmartFactorBase, Stereo) {
  StereoFactor f(unit3);
  f.add(StereoPoint2(), 1);
  f.add(StereoPoint2(), 2);
  EXPECT_LONGS_EQUAL(2 * 3, f.dim());
}
}  // namespace gtsam

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
