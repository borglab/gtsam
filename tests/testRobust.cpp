/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testRobust.cpp
 *  @brief  Unit tests for Robust loss functions
 *  @author Fan Jiang
 *  @author Yetong Zhang
 *  @date   Apr 7, 2022
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

TEST(RobustNoise, loss) {
  // Keys.
  gtsam::Key x1_key = 1;
  gtsam::Key x2_key = 2;

  auto gm = noiseModel::mEstimator::GemanMcClure::Create(1.0);
  auto noise = noiseModel::Robust::Create(gm, noiseModel::Unit::Create(1));

  auto factor = PriorFactor<double>(x1_key, 0.0, noise);
  auto between_factor = BetweenFactor<double>(x1_key, x2_key, 0.0, noise);

  Values values;
  values.insert(x1_key, 10.0);
  values.insert(x2_key, 0.0);

  EXPECT_DOUBLES_EQUAL(0.49505, factor.error(values), 1e-5);
  EXPECT_DOUBLES_EQUAL(0.49505, between_factor.error(values), 1e-5);
  EXPECT_DOUBLES_EQUAL(0.49505, gm->loss(10.0), 1e-5);
}

int main() {
  TestResult tr;

  return TestRegistry::runAllTests(tr);
}
