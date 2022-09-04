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

#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
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
using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::X;

class Robot {
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

  HybridValues delta = robot.optimize();

  delta.continuous().print("delta update:");
  if (delta.discrete()[M(0)] == 0) {
    std::cout << "The robot is stationary!" << std::endl;
  } else {
    std::cout << "The robot has moved!" << std::endl;
  }
  EXPECT_LONGS_EQUAL(0, delta.discrete()[M(0)]);
}

/* ****************************************************************************/
TEST(Estimation, MovingRobot) {
  size_t K = 2;
  vector<double> measurements = {0, 2};

  Robot robot(K, measurements);

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
  DiscreteKeys modes_;
  HybridNonlinearFactorGraph nonlinearFactorGraph_;
  HybridGaussianFactorGraph linearizedFactorGraph_;
  Values linearizationPoint_;

 public:
  /**
   * @brief Construct a new Single Leg object.
   *
   * @param K The number of discrete timesteps
   * @param pims std::vector of preintegrated IMU measurements.
   * @param fk std::vector of forward kinematic measurements for the leg.
   */
  SingleLeg(size_t K, std::vector<Pose2> pims, std::vector<Pose2> fk) {
    // Create DiscreteKeys for binary K modes
    for (size_t k = 0; k < K; k++) {
      modes_.emplace_back(M(k), 2);
    }

    ////// Create hybrid factor graph.

    auto measurement_noise = noiseModel::Isotropic::Sigma(3, 1.0);

    // Add prior on the first pose
    nonlinearFactorGraph_.emplace_nonlinear<PriorFactor<Pose2>>(
        X(0), Pose2(0, 2, 0), measurement_noise);

    // Add measurement factors.
    // These are the preintegrated IMU measurements of the base.
    for (size_t k = 0; k < K - 1; k++) {
      nonlinearFactorGraph_.emplace_nonlinear<BetweenFactor<Pose2>>(
          X(k), X(k + 1), pims.at(k), measurement_noise);
    }

    // Forward kinematics from base X to foot L
    auto fk_noise = noiseModel::Isotropic::Sigma(3, 1.0);
    for (size_t k = 0; k < K; k++) {
      nonlinearFactorGraph_.emplace_nonlinear<BetweenFactor<Pose2>>(
          X(k), L(k), fk.at(k), fk_noise);
    }

    // 2 noise models where moving has a higher covariance.
    auto stance_model = noiseModel::Isotropic::Sigma(3, 1e-2);
    auto swing_model = noiseModel::Isotropic::Sigma(3, 1e2);

    // Add "contact models" for the foot.
    // The idea is that the robot's leg has a tight covariance for stance and
    // loose covariance for swing.
    using ContactFactor = BetweenFactor<Pose2>;

    for (size_t k = 0; k < K - 1; k++) {
      KeyVector keys = {L(k), L(k + 1)};
      DiscreteKeys dkeys{modes_[k], modes_[k + 1]};
      auto stance = boost::make_shared<ContactFactor>(
               keys.at(0), keys.at(1), Pose2(0, 0, 0), stance_model),
           lift = boost::make_shared<ContactFactor>(
               keys.at(0), keys.at(1), Pose2(0, -1, 0), swing_model),
           land = boost::make_shared<ContactFactor>(
               keys.at(0), keys.at(1), Pose2(0, 1, 0), swing_model),
           swing = boost::make_shared<ContactFactor>(
               keys.at(0), keys.at(1), Pose2(1, 0, 0), swing_model);
      // 00 - swing, 01 - land, 10 - toe-off, 11 - stance
      std::vector<boost::shared_ptr<ContactFactor>> components = {swing, land,
                                                                  lift, stance};
      nonlinearFactorGraph_.emplace_hybrid<MixtureFactor>(keys, dkeys,
                                                          components);
    }

    // Create the linearization point.
    for (size_t k = 0; k < K; k++) {
      linearizationPoint_.insert<Pose2>(X(k), Pose2(k, 2, 0));
      linearizationPoint_.insert<Pose2>(L(k), Pose2(0, 0, 0));
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

  Values linearizationPoint() const { return linearizationPoint_; }
};

/* ****************************************************************************/
TEST(Estimation, LeggedRobot) {
  std::vector<Pose2> pims = {Pose2(1, 0, 0)};
  // Leg is in stance throughout
  std::vector<Pose2> fk = {Pose2(0, -2, 0), Pose2(-1, -2, 0)};
  SingleLeg robot(2, pims, fk);

  std::cout << "\n\n\n" << std::endl;
  // robot.print();
  Values initial = robot.linearizationPoint();
  // initial.print();

  HybridValues delta = robot.optimize();
  // delta.print();

  initial.retract(delta.continuous()).print("\n\n=========");
  std::cout << "\n\n\n" << std::endl;
}

/// A robot with a single leg - non-hybrid version.
class SL {
  NonlinearFactorGraph nonlinearFactorGraph_;
  GaussianFactorGraph linearizedFactorGraph_;
  GaussianBayesNet bayesNet_;
  Values linearizationPoint_;

 public:
  /**
   * @brief Construct a new Single Leg object.
   *
   * @param K The number of discrete timesteps
   * @param pims std::vector of preintegrated IMU measurements.
   * @param fk std::vector of forward kinematic measurements for the leg.
   */
  SL(size_t K, const std::vector<Pose2>& pims, const std::vector<Pose2>& fk,
     const std::vector<bool>& contacts) {
    ////// Create hybrid factor graph.

    auto measurement_noise = noiseModel::Isotropic::Sigma(3, 1.0);

    // Add prior on the first pose
    nonlinearFactorGraph_.emplace_shared<PriorFactor<Pose2>>(
        X(0), Pose2(0, 2, 0), measurement_noise);

    // Add measurement factors.
    // These are the preintegrated IMU measurements of the base.
    for (size_t k = 0; k < K - 1; k++) {
      nonlinearFactorGraph_.emplace_shared<BetweenFactor<Pose2>>(
          X(k), X(k + 1), pims.at(k), measurement_noise);
    }

    // Forward kinematics from base X to foot L
    auto fk_noise = noiseModel::Isotropic::Sigma(3, 1.0);
    for (size_t k = 0; k < K; k++) {
      nonlinearFactorGraph_.emplace_shared<BetweenFactor<Pose2>>(
          X(k), L(k), fk.at(k), fk_noise);
    }

    // 2 noise models where moving has a higher covariance.
    auto stance_model = noiseModel::Isotropic::Sigma(3, 1e-4);
    auto swing_model = noiseModel::Isotropic::Sigma(3, 1e8);

    // Add "contact models" for the foot.
    // The idea is that the robot's leg has a tight covariance for stance and
    // loose covariance for swing.
    using ContactFactor = BetweenFactor<Pose2>;

    for (size_t k = 0; k < K - 1; k++) {
      KeyVector keys = {L(k), L(k + 1)};
      ContactFactor::shared_ptr factor;
      if (contacts[k] && contacts[k + 1]) {
        // stance
        std::cout << "stance 11" << std::endl;
        factor = boost::make_shared<ContactFactor>(
            keys.at(0), keys.at(1), Pose2(0, 0, 0), stance_model);
      } else if (contacts[k] && !contacts[k + 1]) {
        // toe-off
        std::cout << "toe-off 10" << std::endl;
        factor = boost::make_shared<ContactFactor>(keys.at(0), keys.at(1),
                                                   Pose2(0, 0, 0), swing_model);
      } else if (!contacts[k] && contacts[k + 1]) {
        // land
        std::cout << "land 01" << std::endl;
        factor = boost::make_shared<ContactFactor>(keys.at(0), keys.at(1),
                                                   Pose2(0, 0, 0), swing_model);
      } else if (!contacts[k] && !contacts[k + 1]) {
        // swing
        std::cout << "swing 00" << std::endl;
        factor = boost::make_shared<ContactFactor>(keys.at(0), keys.at(1),
                                                   Pose2(0, 0, 0), swing_model);
      }

      nonlinearFactorGraph_.push_back(factor);
    }

    // Create the linearization point.
    for (size_t k = 0; k < K; k++) {
      linearizationPoint_.insert<Pose2>(X(k), Pose2(k, 2, 0));
      linearizationPoint_.insert<Pose2>(L(k), Pose2(0, 0, 0));
    }

    linearizedFactorGraph_ =
        *nonlinearFactorGraph_.linearize(linearizationPoint_);
  }

  void print() const {
    nonlinearFactorGraph_.print();
    linearizationPoint_.print();
    linearizedFactorGraph_.print();
  }

  VectorValues optimize() {
    bayesNet_ = *linearizedFactorGraph_.eliminateSequential();

    // bayesNet->print();
    VectorValues delta = bayesNet_.optimize();
    return delta;
  }

  Values linearizationPoint() const { return linearizationPoint_; }
  NonlinearFactorGraph nonlinearFactorGraph() const {
    return nonlinearFactorGraph_;
  }
  GaussianFactorGraph linearizedFactorGraph() const {
    return linearizedFactorGraph_;
  }
  GaussianBayesNet bayesNet() const { return bayesNet_; }
};

/* ****************************************************************************/
TEST(Estimation, LR) {
  std::vector<Pose2> pims = {Pose2(1, 0, 0)};
  // Leg is in stance throughout
  // std::vector<Pose2> fk = {Pose2(0, -2, 0), Pose2(-1, -2, 0)};
  // Leg is in swing
  // std::vector<Pose2> fk = {Pose2(0, -1, 0), Pose2(0, -1, 0)};
  // Leg is in toe-off
  // std::vector<Pose2> fk = {Pose2(0, -2, 0), Pose2(0, -1, 0)};
  // Leg is in land
  std::vector<Pose2> fk = {Pose2(0, -1, 0), Pose2(0, -2, 0)};

  vector<bool> contacts;
  contacts = {1, 1};
  SL robot11(2, pims, fk, contacts);
  VectorValues delta = robot11.optimize();
  // robot11.nonlinearFactorGraph().print();
  std::cout << "Error with optimized delta: " << robot11.bayesNet().error(delta)
            << std::endl;
  robot11.linearizationPoint().retract(delta).print();
  std::cout << "\n===========================\n\n" << std::endl;

  contacts = {1, 0};
  SL robot10(2, pims, fk, contacts);
  delta = robot10.optimize();
  // robot10.nonlinearFactorGraph().print();
  std::cout << "Error with optimized delta: " << robot10.bayesNet().error(delta)
            << std::endl;
  robot10.linearizationPoint().retract(delta).print();
  std::cout << "\n===========================\n\n" << std::endl;

  contacts = {0, 1};
  SL robot01(2, pims, fk, contacts);
  delta = robot01.optimize();
  // robot01.nonlinearFactorGraph().print();
  std::cout << "Error with optimized delta: " << robot01.bayesNet().error(delta)
            << std::endl;
  robot01.linearizationPoint().retract(delta).print();
  std::cout << "\n===========================\n\n" << std::endl;

  contacts = {0, 0};
  SL robot00(2, pims, fk, contacts);
  delta = robot00.optimize();
  // robot00.nonlinearFactorGraph().print();
  std::cout << "Error with optimized delta: " << robot00.bayesNet().error(delta)
            << std::endl;
  robot00.linearizationPoint().retract(delta).print();
  std::cout << "\n===========================\n\n" << std::endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
