/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBatchFactor.cpp
 * @brief Unit tests for BatchFactor class
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <vector>

using namespace gtsam;
using namespace std;

// Define a specific projection factor type for testing
using ProjectionFactor = GenericProjectionFactor<Pose3, Point3, Cal3_S2>;

static std::shared_ptr<Cal3_S2> sharedK = std::make_shared<Cal3_S2>();

/* ************************************************************************* */
TEST(BatchFactor, ConstructorAndLinearize) {
  // 1. Setup data
  std::vector<Key> poses = {Symbol('x', 0)};
  std::vector<Key> points;
  std::vector<Point2> measurements;

  // Create 10 points
  for (int i = 0; i < 10; ++i) {
    points.push_back(Symbol('l', i));
    measurements.push_back(Point2(double(i), double(i)));
  }

  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);

  // 2. Create factors manually
  std::vector<ProjectionFactor> factors;
  for (size_t i = 0; i < points.size(); ++i) {
    factors.emplace_back(measurements[i], noise, poses[0], points[i], sharedK);
  }

  // 3. Construct BatchFactor
  auto batch =
      std::make_shared<BatchFactor<ProjectionFactor, 2>>(std::move(factors));

  // 4. Linearize
  Values values;
  values.insert(Symbol('x', 0), Pose3());
  for (int i = 0; i < 10; ++i) {
    values.insert(Symbol('l', i), Point3(0, 0, 10));
  }

  auto gaussian = batch->linearize(values);
  auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);

  // 5. Verify
  CHECK(jacobian);
  LONGS_EQUAL(20, (long)jacobian->rows());
  LONGS_EQUAL(11, (long)jacobian->size());  // 1 pose + 10 points
}

/* ************************************************************************* */
TEST(BatchFactor, Constructor_Projection) {
  // 1. Setup data
  Key poseKey = Symbol('x', 0);
  std::map<Key, Point2> measurements;

  for (int i = 0; i < 10; ++i) {
    measurements[Symbol('l', i)] = Point2(double(i), double(i));
  }

  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);

  // 2. Construct using Map Constructor (ProjectionFactor style)
  // This should automatically detect the signature: (Measurement, Model, Key1,
  // Key2, K) We pass 'sharedK' as the extra argument.
  auto batch = std::make_shared<BatchFactor<ProjectionFactor, 2>>(
      poseKey, measurements, noise, sharedK);

  // 3. Verify
  Values values;
  values.insert(poseKey, Pose3());
  for (int i = 0; i < 10; ++i) {
    values.insert(Symbol('l', i), Point3(0, 0, 10));
  }

  auto gaussian = batch->linearize(values);
  auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);

  CHECK(jacobian);
  LONGS_EQUAL(20, (long)jacobian->rows());
  LONGS_EQUAL(11, (long)jacobian->size());
}

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>

/* ************************************************************************* */
TEST(BatchFactor, Constructor_Between) {
  // 1. Setup data
  Key key1 = Symbol('x', 0);
  std::map<Key, Pose2> measurements;

  for (int i = 1; i <= 10; ++i) {
    measurements[Symbol('x', i)] = Pose2(1.0, 0.0, 0.0);
  }

  auto noise = noiseModel::Isotropic::Sigma(3, 0.1);

  // 2. Construct using Map Constructor (Standard style)
  // This should detect: (Key1, Key2, Measurement, Model)
  // BetweenFactor takes (Key, Key, Measurement, Model)
  using Between = BetweenFactor<Pose2>;
  auto batch =
      std::make_shared<BatchFactor<Between, 3>>(key1, measurements, noise);

  // 3. Verify
  Values values;
  for (int i = 0; i <= 10; ++i) {
    values.insert(Symbol('x', i), Pose2(double(i), 0.0, 0.0));
  }

  auto gaussian = batch->linearize(values);
  auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);

  CHECK(jacobian);
  LONGS_EQUAL(30, (long)jacobian->rows());  // 10 factors * 3 dim
  LONGS_EQUAL(11, (long)jacobian->size());  // 11 poses
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
