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

#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <vector>

using namespace gtsam;
using namespace std;

// Define a specific projection factor type for testing
using MyProjectionFactor = GenericProjectionFactor<Pose3, Point3, Cal3_S2>;

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
  auto K = std::make_shared<Cal3_S2>();

  // 2. Create factors manually
  std::vector<MyProjectionFactor> factors;
  for (size_t i = 0; i < points.size(); ++i) {
    factors.emplace_back(measurements[i], noise, poses[0], points[i], K);
  }

  // 3. Construct BatchFactor
  auto batch = std::make_shared<BatchFactor<MyProjectionFactor, 2>>(std::move(factors));

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
  // Cols depends on the number of variables and their dimensions.
  // 1 Pose3 (6) + 10 Point3 (3*10) = 36 columns.
  // However, JacobianFactor might store them in a block structure.
  // Let's check the variable count or just existence.
  LONGS_EQUAL(11, (long)jacobian->size()); // 1 pose + 10 points
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
