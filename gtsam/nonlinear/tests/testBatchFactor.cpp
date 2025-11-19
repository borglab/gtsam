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

using ProjectionFactor = GenericProjectionFactor<Pose3, Point3, Cal3_S2>;

static std::shared_ptr<Cal3_S2> sharedK = std::make_shared<Cal3_S2>();

// Define a specific projection factor type for testing
// Define a wrapper to match the constructor signature expected by BatchFactor
// helper
class TestProjectionFactor : public ProjectionFactor {
 public:
  using Base = GenericProjectionFactor<Pose3, Point3, Cal3_S2>;

  TestProjectionFactor() : Base() {}

  // The constructor expected by BatchFactor helper: (Key, Key, Measurement,
  // Model)
  TestProjectionFactor(Key poseKey, Key pointKey, const Point2& measured,
                       const SharedNoiseModel& model)
      : Base(measured, model, poseKey, pointKey, sharedK) {}
};

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
  std::vector<TestProjectionFactor> factors;
  for (size_t i = 0; i < points.size(); ++i) {
    factors.emplace_back(poses[0], points[i], measurements[i], noise);
  }

  // 3. Construct BatchFactor
  auto batch = std::make_shared<BatchFactor<TestProjectionFactor, 2>>(
      std::move(factors));

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
TEST(BatchFactor, ConvenienceConstructor) {
  // 1. Setup data
  std::vector<Key> poses = {Symbol('x', 0)};
  std::vector<Key> points;
  std::vector<Point2> measurements;

  for (int i = 0; i < 10; ++i) {
    points.push_back(Symbol('l', i));
    measurements.push_back(Point2(double(i), double(i)));
  }

  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);

  // 2. Construct using Convenience Constructor (1 camera, 10 points)
  // The helper broadcasts the single pose key against the 10 point keys.
  auto batchConvenience =
      std::make_shared<BatchFactor<TestProjectionFactor, 2>>(
          poses, points, measurements, noise);

  // 3. Construct Manually for comparison
  std::vector<TestProjectionFactor> factors;
  for (size_t i = 0; i < points.size(); ++i) {
    factors.emplace_back(poses[0], points[i], measurements[i], noise);
  }
  auto batchManual = std::make_shared<BatchFactor<TestProjectionFactor, 2>>(
      std::move(factors));

  // 4. Linearize both
  Values values;
  values.insert(Symbol('x', 0), Pose3());
  for (int i = 0; i < 10; ++i) {
    values.insert(Symbol('l', i), Point3(0, 0, 10));
  }

  auto gaussianConvenience = batchConvenience->linearize(values);
  auto gaussianManual = batchManual->linearize(values);

  // 5. Verify they are equal
  CHECK(gaussianConvenience->equals(*gaussianManual, 1e-9));
}

/* ************************************************************************* */
TEST(BatchFactor, FactoryConstructor) {
  // 1. Setup data
  Key poseKey = Symbol('x', 0);
  std::map<Key, Point2> measurements;

  for (int i = 0; i < 10; ++i) {
    measurements[Symbol('l', i)] = Point2(double(i), double(i));
  }

  auto model = noiseModel::Isotropic::Sigma(2, 1.0);

  // 2. Construct using Factory Constructor
  // We iterate over the map, and the lambda creates the factor.
  // This handles the "Argument Order" issue (Measurement first vs Key first)
  // and "Extra Parameters" issue (K) gracefully.
  auto batch = std::make_shared<BatchFactor<ProjectionFactor, 2>>(
      measurements, [&](const std::pair<Key, Point2>& item) {
        // item.first is Key, item.second is Measurement
        return ProjectionFactor(item.second, model, poseKey, item.first,
                                sharedK);
      });

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

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
