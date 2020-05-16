/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testAntiFactor.cpp
 * @brief   Unit test for the AntiFactor
 * @author  Stephen Williams
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/AntiFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( AntiFactor, NegativeHessian)
{
  // The AntiFactor should produce a Hessian Factor with negative matrices

  // Create linearization points
  Pose3 pose1(Rot3(), Point3(0, 0, 0));
  Pose3 pose2(Rot3(), Point3(2, 1, 3));
  Pose3 z(Rot3(), Point3(1, 1, 1));
  SharedNoiseModel sigma(noiseModel::Unit::Create(6));

  // Create a configuration corresponding to the ground truth
  Values values;
  values.insert(1, pose1);
  values.insert(2, pose2);

  // Create a "standard" factor
  BetweenFactor<Pose3>::shared_ptr originalFactor(new BetweenFactor<Pose3>(1, 2, z, sigma));

  // Linearize it into a Jacobian Factor
  GaussianFactor::shared_ptr originalJacobian = originalFactor->linearize(values);

  // Convert it to a Hessian Factor
  HessianFactor::shared_ptr originalHessian = HessianFactor::shared_ptr(new HessianFactor(*originalJacobian));

  // Create the AntiFactor version of the original nonlinear factor
  AntiFactor::shared_ptr antiFactor(new AntiFactor(originalFactor));

  // Linearize the AntiFactor into a Hessian Factor
  GaussianFactor::shared_ptr antiGaussian = antiFactor->linearize(values);
  HessianFactor::shared_ptr antiHessian = boost::dynamic_pointer_cast<HessianFactor>(antiGaussian);

  Matrix expected_information = -originalHessian->information();
  Matrix actual_information = antiHessian->information();
  EXPECT(assert_equal(expected_information, actual_information));

  Vector expected_linear = -originalHessian->linearTerm();
  Vector actual_linear = antiHessian->linearTerm();
  EXPECT(assert_equal(expected_linear, actual_linear));

  double expected_f = -originalHessian->constantTerm();
  double actual_f = antiHessian->constantTerm();
  EXPECT_DOUBLES_EQUAL(expected_f, actual_f, 1e-5);
}

/* ************************************************************************* */
TEST( AntiFactor, EquivalentBayesNet)
{
  // Test the AntiFactor by creating a simple graph and eliminating into a BayesNet
  // Then add an additional factor and the corresponding AntiFactor and eliminate
  // The resulting BayesNet should be identical to the first

  Pose3 pose1(Rot3(), Point3(0, 0, 0));
  Pose3 pose2(Rot3(), Point3(2, 1, 3));
  Pose3 z(Rot3(), Point3(1, 1, 1));
  SharedNoiseModel sigma(noiseModel::Unit::Create(6));

  NonlinearFactorGraph graph;
  graph.addPrior(1, pose1, sigma);
  graph.emplace_shared<BetweenFactor<Pose3> >(1, 2, pose1.between(pose2), sigma);

  // Create a configuration corresponding to the ground truth
  Values values;
  values.insert(1, pose1);
  values.insert(2, pose2);

  // Define an elimination ordering
  Ordering ordering = graph.orderingCOLAMD();

  // Eliminate into a BayesNet
  GaussianFactorGraph lin_graph = *graph.linearize(values);
  GaussianBayesNet::shared_ptr expectedBayesNet = lin_graph.eliminateSequential(ordering);

  // Back-substitute to find the optimal deltas
  VectorValues expectedDeltas = expectedBayesNet->optimize();

  // Add an additional factor between Pose1 and Pose2
  BetweenFactor<Pose3>::shared_ptr f1(new BetweenFactor<Pose3>(1, 2, z, sigma));
  graph.push_back(f1);

  // Add the corresponding AntiFactor between Pose1 and Pose2
  AntiFactor::shared_ptr f2(new AntiFactor(f1));
  graph.push_back(f2);

  // Again, Eliminate into a BayesNet
  GaussianFactorGraph lin_graph1 = *graph.linearize(values);
  GaussianBayesNet::shared_ptr actualBayesNet = lin_graph1.eliminateSequential(ordering);

  // Back-substitute to find the optimal deltas
  VectorValues actualDeltas = actualBayesNet->optimize();

  // Verify the BayesNets are identical
  CHECK(assert_equal(*expectedBayesNet, *actualBayesNet, 1e-5));
  CHECK(assert_equal(expectedDeltas, actualDeltas, 1e-5));
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
