/**
 * @file testIMUSystem
 * @author Alex Cunningham
 */

#include <iostream>

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_unstable/dynamics/IMUFactor.h>
#include <gtsam_unstable/dynamics/FullIMUFactor.h>
#include <gtsam_unstable/dynamics/VelocityConstraint.h>
#include <gtsam_unstable/dynamics/DynamicsPriors.h>

using namespace std;
using namespace gtsam;

const double tol=1e-5;

static const Key x0 = 0, x1 = 1, x2 = 2, x3 = 3, x4 = 4;
static const Vector g = Vector::Unit(3,2)*(-9.81);

/* ************************************************************************* */
TEST(testIMUSystem, instantiations) {
  // just checking for compilation
  PoseRTV x1_v;

  gtsam::SharedNoiseModel model1 = gtsam::noiseModel::Unit::Create(1);
  gtsam::SharedNoiseModel model3 = gtsam::noiseModel::Unit::Create(3);
  gtsam::SharedNoiseModel model6 = gtsam::noiseModel::Unit::Create(6);
  gtsam::SharedNoiseModel model9 = gtsam::noiseModel::Unit::Create(9);

  Vector accel = Vector::Ones(3), gyro = Vector::Ones(3);

  IMUFactor<PoseRTV> imu(accel, gyro, 0.01, x1, x2, model6);
  FullIMUFactor<PoseRTV> full_imu(accel, gyro, 0.01, x1, x2, model9);
  NonlinearEquality<gtsam::PoseRTV> poseHardPrior(x1, x1_v);
  BetweenFactor<gtsam::PoseRTV> odom(x1, x2, x1_v, model9);
  RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> range(x1, x2, 1.0, model1);
  VelocityConstraint constraint(x1, x2, 0.1, 10000);
  PriorFactor<gtsam::PoseRTV> posePrior(x1, x1_v, model9);
  DHeightPrior heightPrior(x1, 0.1, model1);
  VelocityPrior velPrior(x1, Vector::Ones(3), model3);
}

/* ************************************************************************* */
TEST( testIMUSystem, optimize_chain ) {
  // create a simple chain of poses to generate IMU measurements
  const double dt = 1.0;
  PoseRTV pose1,
          pose2(Point3(1.0, 1.0, 0.0), Rot3::Ypr(0.1, 0.0, 0.0), Velocity3(2.0, 2.0, 0.0)),
          pose3(Point3(2.0, 2.0, 0.0), Rot3::Ypr(0.2, 0.0, 0.0), Velocity3(0.0, 0.0, 0.0)),
          pose4(Point3(3.0, 3.0, 0.0), Rot3::Ypr(0.3, 0.0, 0.0), Velocity3(2.0, 2.0, 0.0));

  // create measurements
  SharedDiagonal model = noiseModel::Unit::Create(6);
  Vector6 imu12 = pose1.imuPrediction(pose2, dt);
  Vector6 imu23 = pose2.imuPrediction(pose3, dt);
  Vector6 imu34 = pose3.imuPrediction(pose4, dt);

  // assemble simple graph with IMU measurements and velocity constraints
  NonlinearFactorGraph graph;
  graph += NonlinearEquality<gtsam::PoseRTV>(x1, pose1);
  graph += IMUFactor<PoseRTV>(imu12, dt, x1, x2, model);
  graph += IMUFactor<PoseRTV>(imu23, dt, x2, x3, model);
  graph += IMUFactor<PoseRTV>(imu34, dt, x3, x4, model);
  graph += VelocityConstraint(x1, x2, dt);
  graph += VelocityConstraint(x2, x3, dt);
  graph += VelocityConstraint(x3, x4, dt);

  // ground truth values
  Values true_values;
  true_values.insert(x1, pose1);
  true_values.insert(x2, pose2);
  true_values.insert(x3, pose3);
  true_values.insert(x4, pose4);

  // verify zero error
  EXPECT_DOUBLES_EQUAL(0, graph.error(true_values), 1e-5);

  // initialize with zero values and optimize
  Values values;
  values.insert(x1, PoseRTV());
  values.insert(x2, PoseRTV());
  values.insert(x3, PoseRTV());
  values.insert(x4, PoseRTV());

  Values actual = LevenbergMarquardtOptimizer(graph, values).optimize();
  EXPECT(assert_equal(true_values, actual, tol));
}

/* ************************************************************************* */
TEST( testIMUSystem, optimize_chain_fullfactor ) {
  // create a simple chain of poses to generate IMU measurements
  const double dt = 1.0;
  PoseRTV pose1,
          pose2(Point3(1.0, 0.0, 0.0), Rot3::Ypr(0.0, 0.0, 0.0), Velocity3(1.0, 0.0, 0.0)),
          pose3(Point3(2.0, 0.0, 0.0), Rot3::Ypr(0.0, 0.0, 0.0), Velocity3(1.0, 0.0, 0.0)),
          pose4(Point3(3.0, 0.0, 0.0), Rot3::Ypr(0.0, 0.0, 0.0), Velocity3(1.0, 0.0, 0.0));

  // create measurements
  SharedDiagonal model = noiseModel::Isotropic::Sigma(9, 1.0);
  Vector6 imu12 = pose1.imuPrediction(pose2, dt);
  Vector6 imu23 = pose2.imuPrediction(pose3, dt);
  Vector6 imu34 = pose3.imuPrediction(pose4, dt);

  // assemble simple graph with IMU measurements and velocity constraints
  NonlinearFactorGraph graph;
  graph += NonlinearEquality<gtsam::PoseRTV>(x1, pose1);
  graph += FullIMUFactor<PoseRTV>(imu12, dt, x1, x2, model);
  graph += FullIMUFactor<PoseRTV>(imu23, dt, x2, x3, model);
  graph += FullIMUFactor<PoseRTV>(imu34, dt, x3, x4, model);

  // ground truth values
  Values true_values;
  true_values.insert(x1, pose1);
  true_values.insert(x2, pose2);
  true_values.insert(x3, pose3);
  true_values.insert(x4, pose4);

  // verify zero error
  EXPECT_DOUBLES_EQUAL(0, graph.error(true_values), 1e-5);

  // initialize with zero values and optimize
  Values values;
  values.insert(x1, PoseRTV());
  values.insert(x2, PoseRTV());
  values.insert(x3, PoseRTV());
  values.insert(x4, PoseRTV());

  cout << "Initial Error: " << graph.error(values) << endl; // Initial error is 0.5 - need better prediction model

  Values actual = LevenbergMarquardtOptimizer(graph, values).optimize();
//  EXPECT(assert_equal(true_values, actual, tol)); // FAIL
}

/* ************************************************************************* */
TEST( testIMUSystem, linear_trajectory) {
  // create a linear trajectory of poses
  // and verify simple solution
  const double dt = 1.0;

  PoseRTV start;
  Vector accel = Vector::Unit(3,0)*0.5; // forward force
  Vector gyro = Vector::Unit(3,0)*0.1; // constant rotation
  SharedDiagonal model = noiseModel::Unit::Create(9);

  Values true_traj, init_traj;
  NonlinearFactorGraph graph;

  graph += NonlinearEquality<gtsam::PoseRTV>(x0, start);
  true_traj.insert(x0, start);
  init_traj.insert(x0, start);

  size_t nrPoses = 10;
  PoseRTV cur_pose = start;
  for (size_t i=1; i<nrPoses; ++i) {
    Key xA = i-1, xB = i;
    cur_pose = cur_pose.generalDynamics(accel, gyro, dt);
    graph += FullIMUFactor<PoseRTV>(accel - g, gyro, dt, xA, xB, model);
    true_traj.insert(xB, cur_pose);
    init_traj.insert(xB, PoseRTV());
  }
//  EXPECT_DOUBLES_EQUAL(0, graph.error(true_traj), 1e-5); // FAIL
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
