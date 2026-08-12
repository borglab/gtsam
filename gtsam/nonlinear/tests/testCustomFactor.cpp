/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testCustomFactor.cpp
 * @brief unit tests for CustomFactor used directly from C++
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/CustomFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <vector>

using namespace gtsam;

using symbol_shorthand::X;

namespace {

const std::vector<Point2> kLandmarks{Point2(5.0, 0.0), Point2(2.0, 3.0),
                                     Point2(6.0, 4.0)};
const size_t kNumLandmarks = 3;
const auto kNoiseModel = noiseModel::Isotropic::Sigma(2 * kNumLandmarks, 0.1);

class Residual {
 private:
  std::vector<Point2> measurements_;

 public:
  explicit Residual(std::vector<Point2> measurements)
      : measurements_(std::move(measurements)) {}

  Vector operator()(const CustomFactor& factor, const Values& values,
                    const JacobianVector* jacobians) const {
    const Pose2& pose = values.at<Pose2>(factor.keys()[0]);
    const size_t measurementCount = measurements_.size();
    Vector error = Vector::Zero(2 * measurementCount);
    Matrix* H = nullptr;

    if (jacobians) {
      // CustomFactor exposes a const JacobianVector* so wrapped languages can
      // mutate the shared storage directly. Native C++ callbacks still need to
      // fill the same storage when derivatives are requested.
      auto& mutableJacobians = *const_cast<JacobianVector*>(jacobians);
      H = &mutableJacobians[0];
      H->resize(2 * measurementCount, 3);
    }

    for (size_t i = 0; i < measurementCount; ++i) {
      Point2 predicted;
      Matrix23 poseJacobian;
      if (H) {
        predicted = pose.transformTo(kLandmarks[i], &poseJacobian);
      } else {
        predicted = pose.transformTo(kLandmarks[i]);
      }
      error.segment<2>(2 * i) = predicted - measurements_[i];

      if (H) {
        H->block<2, 3>(2 * i, 0) = poseJacobian;
      }
    }

    return error;
  }
};

std::vector<Point2> measureLandmarks(const Pose2& pose) {
  return std::vector<Point2>{pose.transformTo(kLandmarks[0]),
                             pose.transformTo(kLandmarks[1]),
                             pose.transformTo(kLandmarks[2])};
}

}  // namespace

/* ************************************************************************* */
// Test that the Jacobians computed by CustomFactor match numerical derivatives
TEST(CustomFactor, LandmarkLocalizationJacobians) {
  const Pose2 truePose(1.0, 2.0, 0.3);
  const std::vector<Point2> measurements = measureLandmarks(truePose);
  const auto factor = std::make_shared<CustomFactor>(
      kNoiseModel, KeyVector{X(0)}, Residual(measurements));

  Values values;
  values.insert(X(0), truePose);
  EXPECT(assert_equal(Vector::Zero(2 * kNumLandmarks),
                      factor->unwhitenedError(values), 1e-9));
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
// Test that CustomFactor can be used in an optimization problem
TEST(CustomFactor, LandmarkLocalizationOptimize) {
  // Create Graph
  NonlinearFactorGraph graph;

  const Pose2 truePose0(1.0, 2.0, 0.3);
  const std::vector<Point2> measurements0 = measureLandmarks(truePose0);
  graph.push_back(std::make_shared<CustomFactor>(kNoiseModel, KeyVector{X(0)},
                                                 Residual(measurements0)));

  const Pose2 truePose1(2.5, 1.0, -0.2);
  const std::vector<Point2> measurements1 = measureLandmarks(truePose1);
  graph.push_back(std::make_shared<CustomFactor>(kNoiseModel, KeyVector{X(1)},
                                                 Residual(measurements1)));

  // Create Initial Estimate
  Values initial;
  initial.insert(X(0), Pose2(0.2, 0.5, 0.0));
  initial.insert(X(1), Pose2(3.2, 0.2, 0.1));

  // Optimize
  const Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  EXPECT(assert_equal(truePose0, result.at<Pose2>(X(0)), 1e-5));
  EXPECT(assert_equal(truePose1, result.at<Pose2>(X(1)), 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
