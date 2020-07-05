/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testConcurrentBatchFilter.cpp
 * @brief   Unit tests for the Concurrent Batch Filter
 * @author  Stephen Williams (swilliams8@gatech.edu)
 * @date    Jan 5, 2013
 */

#include <gtsam_unstable/nonlinear/ConcurrentBatchFilter.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

namespace {

// Set up initial pose, odometry difference, loop closure difference, and initialization errors
const Pose3 poseInitial;
const Pose3 poseOdometry( Rot3::RzRyRx(Vector3(0.05, 0.10, -0.75)), Point3(1.0, -0.25, 0.10) );
const Pose3 poseError( Rot3::RzRyRx(Vector3(0.01, 0.02, -0.1)), Point3(0.05, -0.05, 0.02) );

// Set up noise models for the factors
const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
const SharedDiagonal noiseOdometery = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished());
const SharedDiagonal noiseLoop = noiseModel::Diagonal::Sigmas((Vector(6) << 0.25, 0.25, 0.25, 1.0, 1.0, 1.0).finished());

/* ************************************************************************* */
Values BatchOptimize(const NonlinearFactorGraph& graph, const Values& theta, int maxIter = 100) {

  // Create an L-M optimizer
  LevenbergMarquardtParams parameters;
  parameters.maxIterations = maxIter;

  LevenbergMarquardtOptimizer optimizer(graph, theta, parameters);
  Values result = optimizer.optimize();
  return result;
}

/* ************************************************************************* */
NonlinearFactorGraph CalculateMarginals(const NonlinearFactorGraph& factorGraph, const Values& linPoint, const FastList<Key>& keysToMarginalize){


  std::set<Key> KeysToKeep;
  for(const Values::ConstKeyValuePair& key_value: linPoint) { // we cycle over all the keys of factorGraph
    KeysToKeep.insert(key_value.key);
  } // so far we are keeping all keys, but we want to delete the ones that we are going to marginalize
  for(Key key: keysToMarginalize) {
    KeysToKeep.erase(key);
  } // we removed the keys that we have to marginalize

  Ordering ordering;
  for(Key key: keysToMarginalize) {
    ordering.push_back(key);
  } // the keys that we marginalize should be at the beginning in the ordering
  for(Key key: KeysToKeep) {
    ordering.push_back(key);
  }

  GaussianFactorGraph linearGraph = *factorGraph.linearize(linPoint);

  GaussianFactorGraph marginal = *linearGraph.eliminatePartialMultifrontal(KeyVector(keysToMarginalize.begin(), keysToMarginalize.end()), EliminateCholesky).second;

  NonlinearFactorGraph LinearContainerForGaussianMarginals;
  for(const GaussianFactor::shared_ptr& factor: marginal) {
    LinearContainerForGaussianMarginals.push_back(LinearContainerFactor(factor, linPoint));
  }
  return LinearContainerForGaussianMarginals;
}


} // end namespace



/* ************************************************************************* */
TEST( ConcurrentBatchFilter, equals )
{
  // TODO: Test 'equals' more vigorously

  // Create a Concurrent Batch Filter
  LevenbergMarquardtParams parameters1;
  ConcurrentBatchFilter filter1(parameters1);

  // Create an identical Concurrent Batch Filter
  LevenbergMarquardtParams parameters2;
  ConcurrentBatchFilter filter2(parameters2);

  // Create a different Concurrent Batch Filter
  LevenbergMarquardtParams parameters3;
  parameters3.maxIterations = 1;
  ConcurrentBatchFilter filter3(parameters3);

  CHECK(assert_equal(filter1, filter1));
  CHECK(assert_equal(filter1, filter2));
//  CHECK(assert_inequal(filter1, filter3));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, getFactors )
{
  // Create a Concurrent Batch Filter
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Expected graph is empty
  NonlinearFactorGraph expected1;
  // Get actual graph
  NonlinearFactorGraph actual1 = filter.getFactors();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the filter
  NonlinearFactorGraph newFactors1;
  newFactors1.addPrior(1, poseInitial, noisePrior);
  newFactors1.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues1;
  newValues1.insert(1, Pose3());
  newValues1.insert(2, newValues1.at<Pose3>(1).compose(poseOdometry));
  filter.update(newFactors1, newValues1);

  // Expected graph
  NonlinearFactorGraph expected2;
  expected2.push_back(newFactors1);
  // Get actual graph
  NonlinearFactorGraph actual2 = filter.getFactors();
  // Check
  CHECK(assert_equal(expected2, actual2));

  // Add some more factors to the filter
  NonlinearFactorGraph newFactors2;
  newFactors2.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors2.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues2;
  newValues2.insert(3, newValues1.at<Pose3>(2).compose(poseOdometry));
  newValues2.insert(4, newValues2.at<Pose3>(3).compose(poseOdometry));
  filter.update(newFactors2, newValues2);

  // Expected graph
  NonlinearFactorGraph expected3;
  expected3.push_back(newFactors1);
  expected3.push_back(newFactors2);
  // Get actual graph
  NonlinearFactorGraph actual3 = filter.getFactors();
  // Check
  CHECK(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, getLinearizationPoint )
{
  // Create a Concurrent Batch Filter
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = filter.getLinearizationPoint();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the filter
  NonlinearFactorGraph newFactors1;
  newFactors1.addPrior(1, poseInitial, noisePrior);
  newFactors1.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues1;
  newValues1.insert(1, Pose3());
  newValues1.insert(2, newValues1.at<Pose3>(1).compose(poseOdometry));
  filter.update(newFactors1, newValues1);

  // Expected values is equivalent to the provided values only because the provided linearization points were optimal
  Values expected2;
  expected2.insert(newValues1);
  // Get actual linearization point
  Values actual2 = filter.getLinearizationPoint();
  // Check
  CHECK(assert_equal(expected2, actual2));

  // Add some more factors to the filter
  NonlinearFactorGraph newFactors2;
  newFactors2.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors2.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues2;
  newValues2.insert(3, newValues1.at<Pose3>(2).compose(poseOdometry));
  newValues2.insert(4, newValues2.at<Pose3>(3).compose(poseOdometry));
  filter.update(newFactors2, newValues2);

  // Expected values is equivalent to the provided values only because the provided linearization points were optimal
  Values expected3;
  expected3.insert(newValues1);
  expected3.insert(newValues2);
  // Get actual linearization point
  Values actual3 = filter.getLinearizationPoint();
  // Check
  CHECK(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, getOrdering )
{
  // TODO: Think about how to check ordering...
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, getDelta )
{
  // TODO: Think about how to check delta...
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, calculateEstimate )
{
  // Create a Concurrent Batch Filter
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = filter.calculateEstimate();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the filter
  NonlinearFactorGraph newFactors2;
  newFactors2.addPrior(1, poseInitial, noisePrior);
  newFactors2.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues2;
  newValues2.insert(1, Pose3().compose(poseError));
  newValues2.insert(2, newValues2.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  filter.update(newFactors2, newValues2);

  // Expected values from full batch optimization
  NonlinearFactorGraph allFactors2;
  allFactors2.push_back(newFactors2);
  Values allValues2;
  allValues2.insert(newValues2);
  Values expected2 = BatchOptimize(allFactors2, allValues2);
  // Get actual linearization point
  Values actual2 = filter.calculateEstimate();
  // Check
  CHECK(assert_equal(expected2, actual2, 1e-6));

  // Add some more factors to the filter
  NonlinearFactorGraph newFactors3;
  newFactors3.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors3.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues3;
  newValues3.insert(3, newValues2.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues3.insert(4, newValues3.at<Pose3>(3).compose(poseOdometry).compose(poseError));
  filter.update(newFactors3, newValues3);

  // Expected values from full batch optimization
  NonlinearFactorGraph allFactors3;
  allFactors3.push_back(newFactors2);
  allFactors3.push_back(newFactors3);
  Values allValues3;
  allValues3.insert(newValues2);
  allValues3.insert(newValues3);
  Values expected3 = BatchOptimize(allFactors3, allValues3);
  // Get actual linearization point
  Values actual3 = filter.calculateEstimate();
  // Check
  CHECK(assert_equal(expected3, actual3, 1e-6));

  // Also check the single-variable version
  Pose3 expectedPose1 = expected3.at<Pose3>(1);
  Pose3 expectedPose2 = expected3.at<Pose3>(2);
  Pose3 expectedPose3 = expected3.at<Pose3>(3);
  Pose3 expectedPose4 = expected3.at<Pose3>(4);
  // Also check the single-variable version
  Pose3 actualPose1 = filter.calculateEstimate<Pose3>(1);
  Pose3 actualPose2 = filter.calculateEstimate<Pose3>(2);
  Pose3 actualPose3 = filter.calculateEstimate<Pose3>(3);
  Pose3 actualPose4 = filter.calculateEstimate<Pose3>(4);
  // Check
  CHECK(assert_equal(expectedPose1, actualPose1, 1e-6));
  CHECK(assert_equal(expectedPose2, actualPose2, 1e-6));
  CHECK(assert_equal(expectedPose3, actualPose3, 1e-6));
  CHECK(assert_equal(expectedPose4, actualPose4, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, update_empty )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Call update
  filter.update();
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, update_multiple )
{
  // Create a Concurrent Batch Filter
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = filter.calculateEstimate();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the filter
  NonlinearFactorGraph newFactors2;
  newFactors2.addPrior(1, poseInitial, noisePrior);
  newFactors2.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues2;
  newValues2.insert(1, Pose3().compose(poseError));
  newValues2.insert(2, newValues2.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  filter.update(newFactors2, newValues2);

  // Expected values from full batch optimization
  NonlinearFactorGraph allFactors2;
  allFactors2.push_back(newFactors2);
  Values allValues2;
  allValues2.insert(newValues2);
  Values expected2 = BatchOptimize(allFactors2, allValues2);
  // Get actual linearization point
  Values actual2 = filter.calculateEstimate();
  // Check
  CHECK(assert_equal(expected2, actual2, 1e-6));

  // Add some more factors to the filter
  NonlinearFactorGraph newFactors3;
  newFactors3.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors3.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues3;
  newValues3.insert(3, newValues2.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues3.insert(4, newValues3.at<Pose3>(3).compose(poseOdometry).compose(poseError));
  filter.update(newFactors3, newValues3);

  // Expected values from full batch optimization
  NonlinearFactorGraph allFactors3;
  allFactors3.push_back(newFactors2);
  allFactors3.push_back(newFactors3);
  Values allValues3;
  allValues3.insert(newValues2);
  allValues3.insert(newValues3);
  Values expected3 = BatchOptimize(allFactors3, allValues3);
  // Get actual linearization point
  Values actual3 = filter.calculateEstimate();
  // Check
  CHECK(assert_equal(expected3, actual3, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, update_and_marginalize )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Add some factors to the filter
  NonlinearFactorGraph newFactors;
  newFactors.addPrior(1, poseInitial, noisePrior);
  newFactors.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues;
  newValues.insert(1, Pose3().compose(poseError));
  newValues.insert(2, newValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  newValues.insert(3, newValues.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues.insert(4, newValues.at<Pose3>(3).compose(poseOdometry).compose(poseError));

  // Specify a subset of variables to marginalize/move to the smoother
  FastList<Key> keysToMove;
  keysToMove.push_back(1);
  keysToMove.push_back(2);

  // Update the filter
  filter.update(newFactors, newValues, keysToMove);

  // Calculate expected factor graph and values
  Values optimalValues = BatchOptimize(newFactors, newValues);


  NonlinearFactorGraph partialGraph;
  partialGraph.addPrior(1, poseInitial, noisePrior);
  partialGraph.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);
  partialGraph.emplace_shared<BetweenFactor<Pose3> >(2, 3, poseOdometry, noiseOdometery);

  Values partialValues;
  partialValues.insert(1, optimalValues.at<Pose3>(1));
  partialValues.insert(2, optimalValues.at<Pose3>(2));
  partialValues.insert(3, optimalValues.at<Pose3>(3));

  // Create an ordering
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  ordering.push_back(3);

  // Create the set of marginalizable variables
  KeyVector linearIndices {1, 2};

  GaussianFactorGraph linearPartialGraph = *partialGraph.linearize(partialValues);
  GaussianFactorGraph result = *linearPartialGraph.eliminatePartialMultifrontal(linearIndices, EliminateCholesky).second;


  NonlinearFactorGraph expectedGraph;

  // These three lines create three empty factors in expectedGraph:
  // this is done since the equal function in NonlinearFactorGraph also cares about the ordering of the factors
  // and in the actualGraph produced by the HMF we first insert 5 nonlinear factors, then we delete 3 of them, by
  // substituting them with a linear marginal
  expectedGraph.push_back(NonlinearFactor::shared_ptr());
  expectedGraph.push_back(NonlinearFactor::shared_ptr());
  expectedGraph.push_back(NonlinearFactor::shared_ptr());
  // ==========================================================
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(3, 4, poseOdometry, noiseOdometery);

  for(const GaussianFactor::shared_ptr& factor: result) {
    expectedGraph.emplace_shared<LinearContainerFactor>(factor, partialValues);
  }


  // Get the actual factor graph and optimal point
  NonlinearFactorGraph actualGraph = filter.getFactors();
  Values actualValues = filter.calculateEstimate();

  Values expectedValues = optimalValues;

  // Check
  for(Key key: keysToMove) {
    expectedValues.erase(key);
  }


  CHECK(assert_equal(expectedValues, actualValues, 1e-6));
  CHECK(assert_equal(expectedGraph, actualGraph, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, synchronize_0 )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Create empty containers *from* the smoother
  NonlinearFactorGraph smootherSummarization;
  Values smootherSeparatorValues;

  // Create expected values from the filter. For the case where the filter is empty, the expected values are also empty
  NonlinearFactorGraph expectedSmootherFactors, expectedFilterSummarization;
  Values expectedSmootherValues, expectedFilterSeparatorValues;

  // Synchronize
  NonlinearFactorGraph actualSmootherFactors, actualFilterSummarization;
  Values actualSmootherValues, actualFilterSeparatorValues;
  filter.presync();
  filter.synchronize(smootherSummarization, smootherSeparatorValues);
  filter.getSmootherFactors(actualSmootherFactors, actualSmootherValues);
  filter.getSummarizedFactors(actualFilterSummarization, actualFilterSeparatorValues);
  filter.postsync();

  // Check
  CHECK(assert_equal(expectedSmootherFactors, actualSmootherFactors, 1e-6));
  CHECK(assert_equal(expectedSmootherValues, actualSmootherValues, 1e-6));
  CHECK(assert_equal(expectedFilterSummarization, actualFilterSummarization, 1e-6));
  CHECK(assert_equal(expectedFilterSeparatorValues, actualFilterSeparatorValues, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, synchronize_1 )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  parameters.maxIterations = 1;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Insert factors into the filter, but do not marginalize out any variables.
  // The synchronization should still be empty
  NonlinearFactorGraph newFactors;
  newFactors.addPrior(1, poseInitial, noisePrior);
  newFactors.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues;
  newValues.insert(1, Pose3().compose(poseError));
  newValues.insert(2, newValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  filter.update(newFactors, newValues);

  // Create empty containers *from* the smoother
  NonlinearFactorGraph smootherSummarization;
  Values smootherSeparatorValues;

  // Create expected values from the filter. For the case when nothing has been marginalized from the filter, the expected values are empty
  NonlinearFactorGraph expectedSmootherFactors, expectedFilterSummarization;
  Values expectedSmootherValues, expectedFilterSeparatorValues;

  // Synchronize
  NonlinearFactorGraph actualSmootherFactors, actualFilterSummarization;
  Values actualSmootherValues, actualFilterSeparatorValues;
  filter.presync();
  filter.synchronize(smootherSummarization, smootherSeparatorValues);
  filter.getSmootherFactors(actualSmootherFactors, actualSmootherValues);
  filter.getSummarizedFactors(actualFilterSummarization, actualFilterSeparatorValues);
  filter.postsync();

  // Check
  CHECK(assert_equal(expectedSmootherFactors, actualSmootherFactors, 1e-6));
  CHECK(assert_equal(expectedSmootherValues, actualSmootherValues, 1e-6));
  CHECK(assert_equal(expectedFilterSummarization, actualFilterSummarization, 1e-6));
  CHECK(assert_equal(expectedFilterSeparatorValues, actualFilterSeparatorValues, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, synchronize_2 )
{
  std::cout << "*********************** synchronize_2 ************************" << std::endl;
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  //parameters.maxIterations = 1;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Insert factors into the filter, and marginalize out one variable.
  // There should not be information transmitted to the smoother from the filter
  NonlinearFactorGraph newFactors;
  NonlinearFactor::shared_ptr factor1(new PriorFactor<Pose3>(1, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor2(new BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  newFactors.push_back(factor1);
  newFactors.push_back(factor2);
  Values newValues;
  Pose3 value1 = Pose3().compose(poseError);
  Pose3 value2 = value1.compose(poseOdometry).compose(poseError);
  newValues.insert(1, value1);
  newValues.insert(2, value2);
  FastList<Key> keysToMove;
  keysToMove.push_back(1);
  filter.update(newFactors, newValues, keysToMove);
  // this will not work, as in the filter only remains node 2, while 1 was marginalized out
  // Values optimalValues = filter.calculateEstimate();

  Values optimalValues = BatchOptimize(newFactors, newValues);

  // Create empty containers *from* the smoother
  NonlinearFactorGraph smootherSummarization;
  Values smootherSeparatorValues;


  // Create expected values from the filter.
  // The smoother factors include any factor adjacent to a marginalized variable
  NonlinearFactorGraph expectedSmootherFactors;
  expectedSmootherFactors.push_back(factor1);
  expectedSmootherFactors.push_back(factor2);
  Values expectedSmootherValues;
  // We only pass linearization points for the marginalized variables
  expectedSmootherValues.insert(1, optimalValues.at<Pose3>(1));

  // The filter summarization is the remaining factors from marginalizing out the requested variable
  // In the current example, after marginalizing out 1, the filter only contains the separator (2), with
  // no nonlinear factor attached to it, therefore no filter summarization needs to be passed to the smoother
  NonlinearFactorGraph expectedFilterSummarization;
  Values expectedFilterSeparatorValues;
  expectedFilterSeparatorValues.insert(2, optimalValues.at<Pose3>(2));

  // Synchronize
  NonlinearFactorGraph actualSmootherFactors, actualFilterSummarization;
  Values actualSmootherValues, actualFilterSeparatorValues;
  filter.presync();
  filter.synchronize(smootherSummarization, smootherSeparatorValues);
  filter.getSmootherFactors(actualSmootherFactors, actualSmootherValues);
  filter.getSummarizedFactors(actualFilterSummarization, actualFilterSeparatorValues);
  filter.postsync();

  // Check
  CHECK(assert_equal(expectedSmootherFactors, actualSmootherFactors, 1e-6));
  CHECK(assert_equal(expectedSmootherValues, actualSmootherValues, 1e-6));
  CHECK(assert_equal(expectedFilterSummarization, actualFilterSummarization, 1e-6));
  CHECK(assert_equal(expectedFilterSeparatorValues, actualFilterSeparatorValues, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, synchronize_3 )
{
  std::cout << "*********************** synchronize_3 ************************" << std::endl;
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  //parameters.maxIterations = 1;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Insert factors into the filter, and marginalize out one variable.
  // There should not be information transmitted to the smoother from the filter
  NonlinearFactorGraph newFactors;
  NonlinearFactor::shared_ptr factor1(new PriorFactor<Pose3>(1, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor2(new BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  NonlinearFactor::shared_ptr factor3(new BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors.push_back(factor1);
  newFactors.push_back(factor2);
  newFactors.push_back(factor3);

  Values newValues;
  Pose3 value1 = Pose3().compose(poseError);
  Pose3 value2 = value1.compose(poseOdometry).compose(poseError);
  Pose3 value3 = value2.compose(poseOdometry).compose(poseError);
  newValues.insert(1, value1);
  newValues.insert(2, value2);
  newValues.insert(3, value3);

  FastList<Key> keysToMove;
  keysToMove.push_back(1);
  // we add factors to the filter while marginalizing node 1
  filter.update(newFactors, newValues, keysToMove);

  Values optimalValues = BatchOptimize(newFactors, newValues);

  // In this example the smoother is empty
  // Create empty containers *from* the smoother
  NonlinearFactorGraph smootherSummarization;
  Values smootherSeparatorValues;

  // Create expected values from the filter.
  // The smoother factors include any factor adjacent to a marginalized variable
  NonlinearFactorGraph expectedSmootherFactors;
  expectedSmootherFactors.push_back(factor1);
  expectedSmootherFactors.push_back(factor2);
  Values expectedSmootherValues;
  // We only pass linearization points for the marginalized variables
  expectedSmootherValues.insert(1, optimalValues.at<Pose3>(1));

  // In the current example, after marginalizing out 1, the filter contains the separator 2 and node 3, with
  // a nonlinear factor attached to them
  // Why there is no summarization from filter ????
  NonlinearFactorGraph expectedFilterSummarization;
  Values expectedFilterSeparatorValues;
  expectedFilterSeparatorValues.insert(2, optimalValues.at<Pose3>(2));
  // ------------------------------------------------------------------------------
  NonlinearFactorGraph partialGraph;
  partialGraph.push_back(factor3);

  Values partialValues;
  partialValues.insert(2, optimalValues.at<Pose3>(2));
  partialValues.insert(3, optimalValues.at<Pose3>(3));

  FastList<Key> keysToMarginalize;
  keysToMarginalize.push_back(3);

  expectedFilterSummarization = CalculateMarginals(partialGraph, partialValues, keysToMarginalize);
  // ------------------------------------------------------------------------------
  // Synchronize
  NonlinearFactorGraph actualSmootherFactors, actualFilterSummarization;
  Values actualSmootherValues, actualFilterSeparatorValues;
  filter.presync();
  filter.synchronize(smootherSummarization, smootherSeparatorValues);
  filter.getSmootherFactors(actualSmootherFactors, actualSmootherValues);
  filter.getSummarizedFactors(actualFilterSummarization, actualFilterSeparatorValues);
  filter.postsync();

  // Check
  CHECK(assert_equal(expectedSmootherFactors, actualSmootherFactors, 1e-6));
  CHECK(assert_equal(expectedSmootherValues, actualSmootherValues, 1e-6));
  CHECK(assert_equal(expectedFilterSummarization, actualFilterSummarization, 1e-6));
  CHECK(assert_equal(expectedFilterSeparatorValues, actualFilterSeparatorValues, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, synchronize_4 )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  parameters.maxIterations = 1;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Insert factors into the filter, and marginalize out one variable.
  // There should not be information transmitted to the smoother from the filter
  NonlinearFactorGraph newFactors;
  NonlinearFactor::shared_ptr factor1(new PriorFactor<Pose3>(1, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor2(new BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  NonlinearFactor::shared_ptr factor3(new BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors.push_back(factor1);
  newFactors.push_back(factor2);
  newFactors.push_back(factor3);

  Values newValues;
  Pose3 value1 = Pose3().compose(poseError);
  Pose3 value2 = value1.compose(poseOdometry).compose(poseError);
  Pose3 value3 = value2.compose(poseOdometry).compose(poseError);
  newValues.insert(1, value1);
  newValues.insert(2, value2);
  newValues.insert(3, value3);

  FastList<Key> keysToMove;
  keysToMove.push_back(1);
  // we add factors to the filter while marginalizing node 1
  filter.update(newFactors, newValues, keysToMove);

  Values optimalValuesFilter = BatchOptimize(newFactors, newValues,1);

  // In this example the smoother contains a between factor and a prior factor
  // COMPUTE SUMMARIZATION ON THE SMOOTHER SIDE
  NonlinearFactorGraph smootherSummarization;
  Values smootherSeparatorValues;

  // Create expected values from the filter.
  // The smoother factors include any factor adjacent to a marginalized variable
  NonlinearFactorGraph expectedSmootherFactors;
  expectedSmootherFactors.push_back(factor1);
  expectedSmootherFactors.push_back(factor2);
  Values expectedSmootherValues;
  // We only pass linearization points for the marginalized variables
  expectedSmootherValues.insert(1, optimalValuesFilter.at<Pose3>(1));

  // COMPUTE SUMMARIZATION ON THE FILTER SIDE
  // In the current example, after marginalizing out 1, the filter contains the separator 2 and node 3, with
  // a nonlinear factor attached to them
  // Why there is no summarization from filter ????
  NonlinearFactorGraph expectedFilterSummarization;
  Values expectedFilterSeparatorValues;
  expectedFilterSeparatorValues.insert(2, optimalValuesFilter.at<Pose3>(2));
  // ------------------------------------------------------------------------------
  NonlinearFactorGraph partialGraphFilter;
  partialGraphFilter.push_back(factor3);

  Values partialValuesFilter;
  partialValuesFilter.insert(2, optimalValuesFilter.at<Pose3>(2));
  partialValuesFilter.insert(3, optimalValuesFilter.at<Pose3>(3));

  // Create an ordering
  Ordering orderingFilter;
  orderingFilter.push_back(3);
  orderingFilter.push_back(2);

  FastList<Key> keysToMarginalize;
  keysToMarginalize.push_back(3);

  expectedFilterSummarization = CalculateMarginals(partialGraphFilter, partialValuesFilter, keysToMarginalize);
  // ------------------------------------------------------------------------------
  // Synchronize
  // This is only an information compression/exchange: to actually incorporate the info we should call update
  NonlinearFactorGraph actualSmootherFactors, actualFilterSummarization;
  Values actualSmootherValues, actualFilterSeparatorValues;
  filter.presync();
  filter.synchronize(smootherSummarization, smootherSeparatorValues);
  filter.getSmootherFactors(actualSmootherFactors, actualSmootherValues);
  filter.getSummarizedFactors(actualFilterSummarization, actualFilterSeparatorValues);
  filter.postsync();


  // Check
  CHECK(assert_equal(expectedSmootherFactors, actualSmootherFactors, 1e-6));
  CHECK(assert_equal(expectedSmootherValues, actualSmootherValues, 1e-6));
  CHECK(assert_equal(expectedFilterSummarization, actualFilterSummarization, 1e-6));
  CHECK(assert_equal(expectedFilterSeparatorValues, actualFilterSeparatorValues, 1e-6));
}


/* ************************************************************************* */
TEST( ConcurrentBatchFilter, synchronize_5 )
{
  std::cout << "*********************** synchronize_5 ************************" << std::endl;
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  parameters.maxIterations = 1;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Insert factors into the filter, and marginalize out one variable.
  // There should not be information transmitted to the smoother from the filter
  NonlinearFactorGraph newFactors;
  NonlinearFactor::shared_ptr factor1(new PriorFactor<Pose3>(1, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor2(new BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  NonlinearFactor::shared_ptr factor3(new PriorFactor<Pose3>(2, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor4(new BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  NonlinearFactor::shared_ptr factor5(new BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  newFactors.push_back(factor1);
  newFactors.push_back(factor2);
  newFactors.push_back(factor3);
  newFactors.push_back(factor4);
  newFactors.push_back(factor5);

  Values newValues;
  Pose3 value1 = Pose3().compose(poseError);
  Pose3 value2 = value1.compose(poseOdometry).compose(poseError);
  Pose3 value3 = value2.compose(poseOdometry).compose(poseError);
  Pose3 value4 = value3.compose(poseOdometry).compose(poseError);

  newValues.insert(1, value1);
  newValues.insert(2, value2);
  newValues.insert(3, value3);
  newValues.insert(4, value4);

  FastList<Key> keysToMove;
  keysToMove.push_back(1);
  // we add factors to the filter while marginalizing node 1
  filter.update(newFactors, newValues, keysToMove);

  // At the beginning the smoother is empty
  NonlinearFactorGraph smootherSummarization;
  Values smootherSeparatorValues;

  // Synchronize
  // This is only an information compression/exchange: to actually incorporate the info we should call update
  NonlinearFactorGraph actualSmootherFactors, actualFilterSummarization;
  Values actualSmootherValues, actualFilterSeparatorValues;
  filter.presync();
  filter.synchronize(smootherSummarization, smootherSeparatorValues);
  filter.getSmootherFactors(actualSmootherFactors, actualSmootherValues);
  filter.getSummarizedFactors(actualFilterSummarization, actualFilterSeparatorValues);
  filter.postsync();

  NonlinearFactorGraph expectedSmootherFactors;
  expectedSmootherFactors.push_back(factor1);
  expectedSmootherFactors.push_back(factor2);

  Values optimalValues = BatchOptimize(newFactors, newValues, 1);
  Values expectedSmootherValues;
  // Pose3 cast is useless in this case (but we still put it as an example): values and graphs can handle generic
  // geometric objects. You really need the <Pose3> when you need to fill in a Pose3 object with the .at()
  expectedSmootherValues.insert(1,optimalValues.at<Pose3>(1));

  // Check
  CHECK(assert_equal(expectedSmootherFactors, actualSmootherFactors, 1e-6));
  CHECK(assert_equal(expectedSmootherValues, actualSmootherValues, 1e-6));

  // at this point the filter contains: nodes 2 3 4 and factors 3 4 5 + marginal on 2
  Values optimalValues2 = BatchOptimize(filter.getFactors(),filter.getLinearizationPoint(),1);

  FastList<Key> keysToMove2;
  keysToMove2.push_back(2);
  newFactors.resize(0);
  newValues.clear();
  // we add factors to the filter while marginalizing node 1
  filter.update(newFactors, newValues, keysToMove2);


  // At the beginning the smoother is empty
  NonlinearFactorGraph smootherSummarization2;
  Values smootherSeparatorValues2;

  // ------------------------------------------------------------------------------
  // We fake the computation of the smoother separator
  // currently the smoother contains factor 1 and 2 and node 1 and 2

  NonlinearFactorGraph partialGraph;
  partialGraph.push_back(factor1);
  partialGraph.push_back(factor2);

  // we also assume that the smoother received an extra factor (e.g., a prior on 1)
  partialGraph.push_back(factor1);

  Values partialValues;
  // Batch optimization updates all linearization points but the ones of the separator
  // In this case, we start with no separator (everything is in the filter), therefore,
  // we update all linearization point
  partialValues.insert(2, optimalValues.at(2)); //<-- does not actually exist
  //The linearization point of 1 is controlled by the smoother and
  // we are artificially setting that to something different to what was in the filter
  partialValues.insert(1, Pose3().compose(poseError.inverse()));

  FastList<Key> keysToMarginalize;
  keysToMarginalize.push_back(1);

  smootherSummarization2 = CalculateMarginals(partialGraph, partialValues, keysToMarginalize);
  smootherSeparatorValues2.insert(2, partialValues.at(2));

  // ------------------------------------------------------------------------------
  // Synchronize
  // This is only an information compression/exchange: to actually incorporate the info we should call update
  NonlinearFactorGraph actualSmootherFactors2, actualFilterSummarization2;
  Values actualSmootherValues2, actualFilterSeparatorValues2;
  filter.presync();
  filter.synchronize(smootherSummarization2, smootherSeparatorValues2);
  filter.getSmootherFactors(actualSmootherFactors2, actualSmootherValues2);
  filter.getSummarizedFactors(actualFilterSummarization2, actualFilterSeparatorValues2);
  filter.postsync();

  NonlinearFactorGraph expectedSmootherFactors2;
  expectedSmootherFactors2.push_back(factor3);
  expectedSmootherFactors2.push_back(factor4);

  Values expectedSmootherValues2;
  expectedSmootherValues2.insert(2, optimalValues.at(2));

  // Check
  CHECK(assert_equal(expectedSmootherFactors2, actualSmootherFactors2, 1e-6));
  CHECK(assert_equal(expectedSmootherValues2, actualSmootherValues2, 1e-6));


  //   In this example the smoother contains a between factor and a prior factor
  //   COMPUTE SUMMARIZATION ON THE FILTER SIDE
  // ------------------------------------------------------------------------------
  // This cannot be nonempty for the first call to synchronize
  NonlinearFactorGraph partialGraphFilter;
  partialGraphFilter.push_back(factor5);


  Values partialValuesFilter;
  partialValuesFilter.insert(3, optimalValues2.at(3));
  partialValuesFilter.insert(4, optimalValues2.at(4));

  FastList<Key> keysToMarginalize2;
  keysToMarginalize2.push_back(4);

  NonlinearFactorGraph expectedFilterSummarization2 = CalculateMarginals(partialGraphFilter, partialValuesFilter, keysToMarginalize2);
  Values expectedFilterSeparatorValues2;
  expectedFilterSeparatorValues2.insert(3, optimalValues2.at(3));

  CHECK(assert_equal(expectedFilterSummarization2, actualFilterSummarization2, 1e-6));
  CHECK(assert_equal(expectedFilterSeparatorValues2, actualFilterSeparatorValues2, 1e-6));

  // Now we should check that the smooother summarization on the old separator is correctly propagated
  // on the new separator by the filter
  NonlinearFactorGraph partialGraphTransition;
  partialGraphTransition.push_back(factor3);
  partialGraphTransition.push_back(factor4);
  partialGraphTransition.push_back(smootherSummarization2);

  Values partialValuesTransition;
  partialValuesTransition.insert(2,optimalValues.at(2));
  partialValuesTransition.insert(3,optimalValues2.at(3));

  FastList<Key> keysToMarginalize3;
  keysToMarginalize3.push_back(2);

  NonlinearFactorGraph expectedFilterGraph;

  // The assert equal will check if the expected and the actual graphs are the same, taking into account
  // orders of the factors, and empty factors:
  // in the filter we originally had 5 factors, and by marginalizing 1 and 2 we got rid of factors 1 2 3 4,
  // therefore in the expected Factor we should include 4 empty factors.
  // Note that the unit test will fail also if we change the order of the factors, due to the definition of assert_equal
  NonlinearFactor::shared_ptr factorEmpty;
  expectedFilterGraph.push_back(factorEmpty);
  expectedFilterGraph.push_back(factorEmpty);
  expectedFilterGraph.push_back(factorEmpty);
  expectedFilterGraph.push_back(factorEmpty);
  expectedFilterGraph.push_back(factor5);
  expectedFilterGraph.push_back(CalculateMarginals(partialGraphTransition, partialValuesTransition, keysToMarginalize3));

  NonlinearFactorGraph actualFilterGraph;
  actualFilterGraph = filter.getFactors();

  CHECK(assert_equal(expectedFilterGraph, actualFilterGraph, 1e-6));
}


///* ************************************************************************* */
TEST( ConcurrentBatchFilter, CalculateMarginals_1 )
{
  // We compare the manual computation of the linear marginals from a factor graph, with the function CalculateMarginals
  NonlinearFactor::shared_ptr factor1(new PriorFactor<Pose3>(1, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor2(new BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  NonlinearFactor::shared_ptr factor3(new PriorFactor<Pose3>(2, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor4(new BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));

  NonlinearFactorGraph factorGraph;
  factorGraph.push_back(factor1);
  factorGraph.push_back(factor2);
  factorGraph.push_back(factor3);
  factorGraph.push_back(factor4);

  Values newValues;
  Pose3 value1 = Pose3().compose(poseError);
  Pose3 value2 = value1.compose(poseOdometry).compose(poseError);
  Pose3 value3 = value2.compose(poseOdometry).compose(poseError);

  newValues.insert(1, value1);
  newValues.insert(2, value2);
  newValues.insert(3, value3);

  // We first manually
  // Create an ordering
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  ordering.push_back(3);

  GaussianFactorGraph linearFactorGraph = *factorGraph.linearize(newValues);

  // Create the set of marginalizable variables
  KeyVector linearIndices {1};

  GaussianFactorGraph result = *linearFactorGraph.eliminatePartialMultifrontal(linearIndices, EliminateCholesky).second;

  NonlinearFactorGraph expectedMarginals;
  for(const GaussianFactor::shared_ptr& factor: result) {
    expectedMarginals.push_back(LinearContainerFactor(factor, newValues));

  }

  FastList<Key> keysToMarginalize;
  keysToMarginalize.push_back(1);
  NonlinearFactorGraph actualMarginals;
  actualMarginals = CalculateMarginals(factorGraph, newValues, keysToMarginalize);

  // Check
  CHECK(assert_equal(expectedMarginals, actualMarginals, 1e-6));
}

///* ************************************************************************* */
TEST( ConcurrentBatchFilter, CalculateMarginals_2 )
{
  // We compare the manual computation of the linear marginals from a factor graph, with the function CalculateMarginals
  NonlinearFactor::shared_ptr factor1(new PriorFactor<Pose3>(1, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor2(new BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  NonlinearFactor::shared_ptr factor3(new PriorFactor<Pose3>(2, poseInitial, noisePrior));
  NonlinearFactor::shared_ptr factor4(new BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));

  NonlinearFactorGraph factorGraph;
  factorGraph.push_back(factor1);
  factorGraph.push_back(factor2);
  factorGraph.push_back(factor3);
  factorGraph.push_back(factor4);

  Values newValues;
  Pose3 value1 = Pose3().compose(poseError);
  Pose3 value2 = value1.compose(poseOdometry).compose(poseError);
  Pose3 value3 = value2.compose(poseOdometry).compose(poseError);

  newValues.insert(1, value1);
  newValues.insert(2, value2);
  newValues.insert(3, value3);

  // We first manually
  // Create an ordering
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  ordering.push_back(3);

  GaussianFactorGraph linearFactorGraph = *factorGraph.linearize(newValues);

  // Create the set of marginalizable variables
  KeyVector linearIndices {1, 2};

  GaussianFactorGraph result = *linearFactorGraph.eliminatePartialMultifrontal(linearIndices, EliminateCholesky).second;

  NonlinearFactorGraph expectedMarginals;
  for(const GaussianFactor::shared_ptr& factor: result) {
    expectedMarginals.push_back(LinearContainerFactor(factor, newValues));

  }

  FastList<Key> keysToMarginalize;
  keysToMarginalize.push_back(1);
  keysToMarginalize.push_back(2);
  NonlinearFactorGraph actualMarginals;
  actualMarginals = CalculateMarginals(factorGraph, newValues, keysToMarginalize);

  // Check
  CHECK(assert_equal(expectedMarginals, actualMarginals, 1e-6));
}

///* ************************************************************************* */
TEST( ConcurrentBatchFilter, removeFactors_topology_1 )
{
  std::cout << "*********************** removeFactors_topology_1 ************************" << std::endl;

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Add some factors to the filter
  NonlinearFactorGraph newFactors;
  newFactors.addPrior(1, poseInitial, noisePrior);
  newFactors.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));

  Values newValues;
  newValues.insert(1, Pose3().compose(poseError));
  newValues.insert(2, newValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  newValues.insert(3, newValues.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues.insert(4, newValues.at<Pose3>(3).compose(poseOdometry).compose(poseError));

  // Specify a subset of variables to marginalize/move to the smoother
  FastList<Key> keysToMove;

  // Update the filter: add all factors
  filter.update(newFactors, newValues, keysToMove);

  // factor we want to remove
  // NOTE: we can remove factors, paying attention that the remaining graph remains connected
  // we remove a single factor, the number 1, which is a BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery);
  std::vector<size_t> removeFactorIndices(1,1);

  // Add no factors to the filter (we only want to test the removal)
  NonlinearFactorGraph noFactors;
  Values noValues;
  filter.update(noFactors, noValues, keysToMove, removeFactorIndices);

  NonlinearFactorGraph actualGraph = filter.getFactors();

  NonlinearFactorGraph expectedGraph;
  expectedGraph.addPrior(1, poseInitial, noisePrior);
  // we removed this one: expectedGraph.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);
  // we should add an empty one, so that the ordering and labeling of the factors is preserved
  expectedGraph.push_back(NonlinearFactor::shared_ptr());
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(2, 3, poseOdometry, noiseOdometery);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(3, 4, poseOdometry, noiseOdometery);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);

  CHECK(assert_equal(expectedGraph, actualGraph, 1e-6));
}

///* ************************************************************************* */
TEST( ConcurrentBatchFilter, removeFactors_topology_2 )
{
  std::cout << "*********************** removeFactors_topology_2 ************************" << std::endl;
  // we try removing the last factor


  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Add some factors to the filter
  NonlinearFactorGraph newFactors;
  newFactors.addPrior(1, poseInitial, noisePrior);
  newFactors.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));

  Values newValues;
  newValues.insert(1, Pose3().compose(poseError));
  newValues.insert(2, newValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  newValues.insert(3, newValues.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues.insert(4, newValues.at<Pose3>(3).compose(poseOdometry).compose(poseError));

  // Specify a subset of variables to marginalize/move to the smoother
  FastList<Key> keysToMove;

  // Update the filter: add all factors
  filter.update(newFactors, newValues, keysToMove);

  // factor we want to remove
  // NOTE: we can remove factors, paying attention that the remaining graph remains connected
  // we remove a single factor, the number 1, which is a BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery);
  std::vector<size_t> removeFactorIndices(1,4);

  // Add no factors to the filter (we only want to test the removal)
  NonlinearFactorGraph noFactors;
  Values noValues;
  filter.update(noFactors, noValues, keysToMove, removeFactorIndices);

  NonlinearFactorGraph actualGraph = filter.getFactors();

  NonlinearFactorGraph expectedGraph;
  expectedGraph.addPrior(1, poseInitial, noisePrior);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(2, 3, poseOdometry, noiseOdometery);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(3, 4, poseOdometry, noiseOdometery);
  // we removed this one: expectedGraph.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);
  // we should add an empty one, so that the ordering and labeling of the factors is preserved
  expectedGraph.push_back(NonlinearFactor::shared_ptr());

  CHECK(assert_equal(expectedGraph, actualGraph, 1e-6));
}


///* ************************************************************************* */
TEST( ConcurrentBatchFilter, removeFactors_topology_3 )
{
  std::cout << "*********************** removeFactors_topology_3 ************************" << std::endl;
  // we try removing the first factor

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Add some factors to the filter
  NonlinearFactorGraph newFactors;
  newFactors.addPrior(1, poseInitial, noisePrior);
  newFactors.addPrior(1, poseInitial, noisePrior);
  newFactors.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));

  Values newValues;
  newValues.insert(1, Pose3().compose(poseError));
  newValues.insert(2, newValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  newValues.insert(3, newValues.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues.insert(4, newValues.at<Pose3>(3).compose(poseOdometry).compose(poseError));

  // Specify a subset of variables to marginalize/move to the smoother
  FastList<Key> keysToMove;

  // Update the filter: add all factors
  filter.update(newFactors, newValues, keysToMove);

  // factor we want to remove
  // NOTE: we can remove factors, paying attention that the remaining graph remains connected
  // we remove a single factor, the number 0, which is a BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery);
  std::vector<size_t> removeFactorIndices(1,0);

  // Add no factors to the filter (we only want to test the removal)
  NonlinearFactorGraph noFactors;
  Values noValues;
  filter.update(noFactors, noValues, keysToMove, removeFactorIndices);

  NonlinearFactorGraph actualGraph = filter.getFactors();

  NonlinearFactorGraph expectedGraph;
  // we should add an empty one, so that the ordering and labeling of the factors is preserved
  expectedGraph.push_back(NonlinearFactor::shared_ptr());
  expectedGraph.addPrior(1, poseInitial, noisePrior);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(2, 3, poseOdometry, noiseOdometery);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(3, 4, poseOdometry, noiseOdometery);

  CHECK(assert_equal(expectedGraph, actualGraph, 1e-6));
}

///* ************************************************************************* */
TEST( ConcurrentBatchFilter, removeFactors_values )
{
  std::cout << "*********************** removeFactors_values ************************" << std::endl;
  // we try removing the last factor

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  ConcurrentBatchFilter filter(parameters);

  // Add some factors to the filter
  NonlinearFactorGraph newFactors;
  newFactors.addPrior(1, poseInitial, noisePrior);
  newFactors.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);
  newFactors.emplace_shared<BetweenFactor<Pose3> >(2, 3, poseOdometry, noiseOdometery);
  newFactors.emplace_shared<BetweenFactor<Pose3> >(3, 4, poseOdometry, noiseOdometery);
  newFactors.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);

  Values newValues;
  newValues.insert(1, Pose3().compose(poseError));
  newValues.insert(2, newValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  newValues.insert(3, newValues.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues.insert(4, newValues.at<Pose3>(3).compose(poseOdometry).compose(poseError));

  // Specify a subset of variables to marginalize/move to the smoother
  FastList<Key> keysToMove;

  // Update the filter: add all factors
  filter.update(newFactors, newValues, keysToMove);

  // factor we want to remove
  // NOTE: we can remove factors, paying attention that the remaining graph remains connected
  // we remove a single factor, the number 4, which is a BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery);
  std::vector<size_t> removeFactorIndices(1,4);

  // Add no factors to the filter (we only want to test the removal)
  NonlinearFactorGraph noFactors;
  Values noValues;
  filter.update(noFactors, noValues, keysToMove, removeFactorIndices);

  NonlinearFactorGraph actualGraph = filter.getFactors();
  Values actualValues = filter.calculateEstimate();

  // note: factors are removed before the optimization
  NonlinearFactorGraph expectedGraph;
  expectedGraph.addPrior(1, poseInitial, noisePrior);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(2, 3, poseOdometry, noiseOdometery);
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(3, 4, poseOdometry, noiseOdometery);
  // we removed this one:   expectedGraph.emplace_shared<BetweenFactor<Pose3> >(1, 2, poseOdometry, noiseOdometery);
  // we should add an empty one, so that the ordering and labeling of the factors is preserved
  expectedGraph.push_back(NonlinearFactor::shared_ptr());

  // Calculate expected factor graph and values
  Values expectedValues = BatchOptimize(expectedGraph, newValues);

  CHECK(assert_equal(expectedGraph, actualGraph, 1e-6));
  CHECK(assert_equal(expectedValues, actualValues, 1e-6));
}

///* ************************************************************************* */
//TEST( ConcurrentBatchFilter, synchronize_10 )
//{
//  // Create a set of optimizer parameters
//  LevenbergMarquardtParams parameters;
//  parameters.maxIterations = 1;
//
//  // Create a Concurrent Batch Filter
//  ConcurrentBatchFilter filter(parameters);
//
//  // Insert factors into the filter, and marginalize out several variables.
//  // This test places several factors in the smoother side, while leaving
//  // several factors on the filter side
//}
//
///* ************************************************************************* */
//TEST( ConcurrentBatchFilter, synchronize_11 )
//{
//  // Create a set of optimizer parameters
//  LevenbergMarquardtParams parameters;
//  parameters.maxIterations = 1;
//
//  // Create a Concurrent Batch Filter
//  ConcurrentBatchFilter filter(parameters);
//
//  // Insert factors into the filter, and marginalize out several variables.
//
//  // Generate a non-empty smoother update, simulating synchronizing with a non-empty smoother
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
