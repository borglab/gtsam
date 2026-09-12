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
#include <gtsam/geometry/Point2.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/ExtendedPriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
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

/* ************************************************************************* */
namespace scalar_robust {

// Vector loss preserves Gaussian, hard-constraint, and Block loss semantics.
TEST(RobustNoise, VectorLossCompatibility) {
  using namespace noiseModel;
  const Vector2 residual{2, -3};
  const auto diagonal = Diagonal::Sigmas(Vector2{2, 3});
  EXPECT_DOUBLES_EQUAL(1.0, diagonal->loss(residual), 1e-12);
  const auto constrained = Constrained::MixedSigmas(
      Vector2{10, 20}, Vector2{0, 3});
  EXPECT_DOUBLES_EQUAL(20.5, constrained->loss(residual), 1e-12);
  const auto block = Robust::Create(mEstimator::Huber::Create(1.0), diagonal);
  EXPECT_DOUBLES_EQUAL(std::sqrt(2.0) - 0.5, block->loss(residual), 1e-12);
  EXPECT_DOUBLES_EQUAL(block->loss(2.0), block->loss(residual), 1e-12);
}

// Scalar Huber sums component losses instead of applying Huber to the norm.
TEST(RobustNoise, ScalarObjective) {
  const auto model = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(
          1.0, noiseModel::mEstimator::Base::Scalar),
      noiseModel::Unit::Create(2));
  const PriorFactor<Point2> factor(0, Point2(0, 0), model);
  Values values;
  values.insert(0, Point2(2, 2));
  EXPECT_DOUBLES_EQUAL(3.0, factor.error(values), 1e-12);
}

// Objective gradients must match IRLS, including whitening and signed losses.
TEST(RobustNoise, ObjectiveGradient) {
  using namespace noiseModel;
  using Scheme = mEstimator::Base;
  const auto gaussian = Gaussian::SqrtInformation(
      Matrix2{{2.0, 0.5}, {0.0, 0.75}});
  const std::vector<SharedNoiseModel> models{
      gaussian,
      Robust::Create(mEstimator::Huber::Create(1.0, Scheme::Scalar), gaussian),
      Robust::Create(mEstimator::Cauchy::Create(1.0, Scheme::Scalar), gaussian),
      Robust::Create(mEstimator::AsymmetricCauchy::Create(1.0, Scheme::Scalar),
                     gaussian),
      Robust::Create(mEstimator::AsymmetricTukey::Create(4.0, Scheme::Scalar),
                     gaussian),
      Robust::Create(mEstimator::Huber::Create(1.0, Scheme::Block), gaussian)};
  for (const auto& model : models) {
    const PriorFactor<Point2> factor(0, Point2(0, 0), model);
    const ExtendedPriorFactor<Point2> extended(0, Point2(0, 0), model);
    for (const Point2& point : {Point2(2, -0.5), Point2(-2, 0.5)}) {
      Values values;
      values.insert(0, point);
      EXPECT_DOUBLES_EQUAL(model->loss(point), factor.error(values), 1e-12);
      EXPECT_DOUBLES_EQUAL(factor.error(values), extended.error(point), 1e-12);
      EXPECT_DOUBLES_EQUAL(extended.error(point), extended.error(values), 1e-12);
      const auto error = [&factor](const Point2& value) {
        Values state;
        state.insert(0, value);
        return factor.error(state);
      };
      const Matrix12 numerical = numericalDerivative11<double, Point2>(
          std::function<double(const Point2&)>(error), point);
      const Vector2 gradient = factor.linearize(values)->gradientAtZero().at(0);
      CHECK(assert_equal(Vector2(numerical.transpose()), gradient, 1e-7));
    }
  }
}

// Direct extended-prior likelihoods must agree with factor-graph evaluation.
TEST(RobustNoise, ExtendedPriorScalarObjective) {
  const auto model = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(
          1.0, noiseModel::mEstimator::Base::Scalar),
      noiseModel::Diagonal::Sigmas(Vector2{2, 3}));
  const ExtendedPriorFactor<Point2> factor(0, Point2(0, 0), model);
  const Point2 point(4, 6);
  Values values;
  values.insert(0, point);
  EXPECT_DOUBLES_EQUAL(3.0, factor.error(point), 1e-12);
  EXPECT_DOUBLES_EQUAL(factor.error(point), factor.error(values), 1e-12);
}

// Scalar robust step acceptance must converge to the separable objective's MAP.
TEST(RobustNoise, ScalarOptimization) {
  const auto model = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(
          1.0, noiseModel::mEstimator::Base::Scalar),
      noiseModel::Unit::Create(2));
  NonlinearFactorGraph graph;
  graph.addPrior(0, Point2(0, 0), model);
  graph.addPrior(0, Point2(4, 4), noiseModel::Unit::Create(2));
  Values initial;
  initial.insert(0, Point2(0, 0));
  LevenbergMarquardtParams params;
  params.relativeErrorTol = 1e-12;
  params.absoluteErrorTol = 1e-12;
  const Values result =
      LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  CHECK(assert_equal(Point2(3, 3), result.at<Point2>(0), 1e-5));
  EXPECT_DOUBLES_EQUAL(6.0, graph.error(result), 1e-10);
}

}  // namespace scalar_robust
/* ************************************************************************* */

int main() {
  TestResult tr;

  return TestRegistry::runAllTests(tr);
}
