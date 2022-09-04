/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridEstimation.cpp
 * @brief   Unit tests for Hybrid Estimation
 * @author  Varun Agrawal
 */

#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

class Robot {
  size_t K_;
  DiscreteKeys modes_;
  HybridNonlinearFactorGraph nonlinearFactorGraph_;
  HybridGaussianFactorGraph linearizedFactorGraph_;
  Values linearizationPoint_;

 public:
  Robot(size_t K, std::vector<double> measurements) {
    // Create DiscreteKeys for binary K modes
    for (size_t k = 0; k < K; k++) {
      modes_.emplace_back(M(k), 2);
    }

    ////// Create hybrid factor graph.
    // Add measurement factors
    auto measurement_noise = noiseModel::Isotropic::Sigma(1, 1.0);
    for (size_t k = 0; k < K; k++) {
      nonlinearFactorGraph_.emplace_nonlinear<PriorFactor<double>>(
          X(k), measurements.at(k), measurement_noise);
    }

    // 2 noise models where moving has a higher covariance.
    auto still_noise_model = noiseModel::Isotropic::Sigma(1, 1e-2);
    auto moving_noise_model = noiseModel::Isotropic::Sigma(1, 1e2);

    // Add "motion models".
    // The idea is that the robot has a higher "freedom" (aka higher covariance)
    // for movement
    using MotionModel = BetweenFactor<double>;

    for (size_t k = 1; k < K; k++) {
      KeyVector keys = {X(k - 1), X(k)};
      DiscreteKeys dkeys{modes_[k - 1]};
      auto still = boost::make_shared<MotionModel>(X(k - 1), X(k), 0.0,
                                                   still_noise_model),
           moving = boost::make_shared<MotionModel>(X(k - 1), X(k), 0.0,
                                                    moving_noise_model);
      std::vector<boost::shared_ptr<MotionModel>> components = {still, moving};
      nonlinearFactorGraph_.emplace_hybrid<MixtureFactor>(keys, dkeys,
                                                          components);
    }

    // Create the linearization point.
    for (size_t k = 0; k < K; k++) {
      linearizationPoint_.insert<double>(X(k), static_cast<double>(k + 1));
    }

    linearizedFactorGraph_ =
        nonlinearFactorGraph_.linearize(linearizationPoint_);
  }

  void print() const {
    nonlinearFactorGraph_.print();
    linearizationPoint_.print();
    linearizedFactorGraph_.print();
  }

  HybridValues optimize() const {
    Ordering hybridOrdering = linearizedFactorGraph_.getHybridOrdering();
    HybridBayesNet::shared_ptr hybridBayesNet =
        linearizedFactorGraph_.eliminateSequential(hybridOrdering);

    HybridValues delta = hybridBayesNet->optimize();
    return delta;
  }
};

/* ****************************************************************************/
/**
 * I am trying to test if setting the hybrid mixture components to just differ
 * in covariance makes sense. This is done by setting the "moving" covariance to
 * be 1e2 while the "still" covariance is 1e-2.
 */
TEST(Estimation, StillRobot) {
  size_t K = 2;
  vector<double> measurements = {0, 0};

  Robot robot(K, measurements);
  // robot.print();

  HybridValues delta = robot.optimize();

  delta.continuous().print("delta update:");

  if (delta.discrete()[M(0)] == 0) {
    std::cout << "The robot is stationary!" << std::endl;
  } else {
    std::cout << "The robot has moved!" << std::endl;
  }
  EXPECT_LONGS_EQUAL(0, delta.discrete()[M(0)]);
}

TEST(Estimation, MovingRobot) {
  size_t K = 2;
  vector<double> measurements = {0, 2};

  Robot robot(K, measurements);
  // robot.print();

  HybridValues delta = robot.optimize();

  delta.continuous().print("delta update:");

  if (delta.discrete()[M(0)] == 0) {
    std::cout << "The robot is stationary!" << std::endl;
  } else {
    std::cout << "The robot has moved!" << std::endl;
  }
  EXPECT_LONGS_EQUAL(1, delta.discrete()[M(0)]);
}

/// A robot with a single leg.
class SingleLeg {
  size_t K_;
  DiscreteKeys modes_;
  HybridNonlinearFactorGraph nonlinearFactorGraph_;
  HybridGaussianFactorGraph linearizedFactorGraph_;
  Values linearizationPoint_;

 public:
  SingleLeg(size_t K, std::vector<double> measurements) {
    // Create DiscreteKeys for binary K modes
    for (size_t k = 0; k < K; k++) {
      modes_.emplace_back(M(k), 2);
    }

    ////// Create hybrid factor graph.
    // Add measurement factors
    auto measurement_noise = noiseModel::Isotropic::Sigma(1, 1.0);
    for (size_t k = 0; k < K; k++) {
      nonlinearFactorGraph_.emplace_nonlinear<PriorFactor<double>>(
          X(k), measurements.at(k), measurement_noise);
    }

    // 2 noise models where moving has a higher covariance.
    auto still_noise_model = noiseModel::Isotropic::Sigma(1, 1e-2);
    auto moving_noise_model = noiseModel::Isotropic::Sigma(1, 1e2);

    // Add "motion models".
    // The idea is that the robot has a higher "freedom" (aka higher covariance)
    // for movement
    using MotionModel = BetweenFactor<double>;

    for (size_t k = 1; k < K; k++) {
      KeyVector keys = {X(k - 1), X(k)};
      DiscreteKeys dkeys{modes_[k - 1]};
      auto still = boost::make_shared<MotionModel>(X(k - 1), X(k), 0.0,
                                                   still_noise_model),
           moving = boost::make_shared<MotionModel>(X(k - 1), X(k), 0.0,
                                                    moving_noise_model);
      std::vector<boost::shared_ptr<MotionModel>> components = {still, moving};
      nonlinearFactorGraph_.emplace_hybrid<MixtureFactor>(keys, dkeys,
                                                          components);
    }

    // Create the linearization point.
    for (size_t k = 0; k < K; k++) {
      linearizationPoint_.insert<double>(X(k), static_cast<double>(k + 1));
    }

    linearizedFactorGraph_ =
        nonlinearFactorGraph_.linearize(linearizationPoint_);
  }

  void print() const {
    nonlinearFactorGraph_.print();
    linearizationPoint_.print();
    linearizedFactorGraph_.print();
  }

  HybridValues optimize() const {
    Ordering hybridOrdering = linearizedFactorGraph_.getHybridOrdering();
    HybridBayesNet::shared_ptr hybridBayesNet =
        linearizedFactorGraph_.eliminateSequential(hybridOrdering);

    HybridValues delta = hybridBayesNet->optimize();
    return delta;
  }
};

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
