/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testConcurrentBatchSmoother.cpp
 * @brief   Unit tests for the Concurrent Batch Smoother
 * @author  Stephen Williams (swilliams8@gatech.edu)
 * @date    Jan 5, 2013
 */

#include <gtsam_unstable/nonlinear/ConcurrentBatchSmoother.h>
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

} // end namespace




/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, equals )
{
  // TODO: Test 'equals' more vigorously

  // Create a Concurrent Batch Smoother
  LevenbergMarquardtParams parameters1;
  ConcurrentBatchSmoother smoother1(parameters1);

  // Create an identical Concurrent Batch Smoother
  LevenbergMarquardtParams parameters2;
  ConcurrentBatchSmoother smoother2(parameters2);

  // Create a different Concurrent Batch Smoother
  LevenbergMarquardtParams parameters3;
  parameters3.maxIterations = 1;
  ConcurrentBatchSmoother smoother3(parameters3);

  CHECK(assert_equal(smoother1, smoother1));
  CHECK(assert_equal(smoother1, smoother2));
//  CHECK(assert_inequal(smoother1, smoother3));
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, getFactors )
{
  // Create a Concurrent Batch Smoother
  LevenbergMarquardtParams parameters;
  ConcurrentBatchSmoother smoother(parameters);

  // Expected graph is empty
  NonlinearFactorGraph expected1;
  // Get actual graph
  NonlinearFactorGraph actual1 = smoother.getFactors();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the smoother
  NonlinearFactorGraph newFactors1;
  newFactors1.addPrior(1, poseInitial, noisePrior);
  newFactors1.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues1;
  newValues1.insert(1, Pose3());
  newValues1.insert(2, newValues1.at<Pose3>(1).compose(poseOdometry));
  smoother.update(newFactors1, newValues1);

  // Expected graph
  NonlinearFactorGraph expected2;
  expected2.push_back(newFactors1);
  // Get actual graph
  NonlinearFactorGraph actual2 = smoother.getFactors();
  // Check
  CHECK(assert_equal(expected2, actual2));

  // Add some more factors to the smoother
  NonlinearFactorGraph newFactors2;
  newFactors2.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors2.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues2;
  newValues2.insert(3, newValues1.at<Pose3>(2).compose(poseOdometry));
  newValues2.insert(4, newValues2.at<Pose3>(3).compose(poseOdometry));
  smoother.update(newFactors2, newValues2);

  // Expected graph
  NonlinearFactorGraph expected3;
  expected3.push_back(newFactors1);
  expected3.push_back(newFactors2);
  // Get actual graph
  NonlinearFactorGraph actual3 = smoother.getFactors();
  // Check
  CHECK(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, getLinearizationPoint )
{
  // Create a Concurrent Batch Smoother
  LevenbergMarquardtParams parameters;
  ConcurrentBatchSmoother smoother(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = smoother.getLinearizationPoint();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the smoother
  NonlinearFactorGraph newFactors1;
  newFactors1.addPrior(1, poseInitial, noisePrior);
  newFactors1.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues1;
  newValues1.insert(1, Pose3());
  newValues1.insert(2, newValues1.at<Pose3>(1).compose(poseOdometry));
  smoother.update(newFactors1, newValues1);

  // Expected values is equivalent to the provided values only because the provided linearization points were optimal
  Values expected2;
  expected2.insert(newValues1);
  // Get actual linearization point
  Values actual2 = smoother.getLinearizationPoint();
  // Check
  CHECK(assert_equal(expected2, actual2));

  // Add some more factors to the smoother
  NonlinearFactorGraph newFactors2;
  newFactors2.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors2.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues2;
  newValues2.insert(3, newValues1.at<Pose3>(2).compose(poseOdometry));
  newValues2.insert(4, newValues2.at<Pose3>(3).compose(poseOdometry));
  smoother.update(newFactors2, newValues2);

  // Expected values is equivalent to the provided values only because the provided linearization points were optimal
  Values expected3;
  expected3.insert(newValues1);
  expected3.insert(newValues2);
  // Get actual linearization point
  Values actual3 = smoother.getLinearizationPoint();
  // Check
  CHECK(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, getOrdering )
{
  // TODO: Think about how to check ordering...
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, getDelta )
{
  // TODO: Think about how to check delta...
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, calculateEstimate )
{
  // Create a Concurrent Batch Smoother
  LevenbergMarquardtParams parameters;
  ConcurrentBatchSmoother smoother(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = smoother.calculateEstimate();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the smoother
  NonlinearFactorGraph newFactors2;
  newFactors2.addPrior(1, poseInitial, noisePrior);
  newFactors2.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues2;
  newValues2.insert(1, Pose3().compose(poseError));
  newValues2.insert(2, newValues2.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  smoother.update(newFactors2, newValues2);

  // Expected values from full batch optimization
  NonlinearFactorGraph allFactors2;
  allFactors2.push_back(newFactors2);
  Values allValues2;
  allValues2.insert(newValues2);
  Values expected2 = BatchOptimize(allFactors2, allValues2);
  // Get actual linearization point
  Values actual2 = smoother.calculateEstimate();
  // Check
  CHECK(assert_equal(expected2, actual2, 1e-6));

  // Add some more factors to the smoother
  NonlinearFactorGraph newFactors3;
  newFactors3.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors3.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues3;
  newValues3.insert(3, newValues2.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues3.insert(4, newValues3.at<Pose3>(3).compose(poseOdometry).compose(poseError));
  smoother.update(newFactors3, newValues3);

  // Expected values from full batch optimization
  NonlinearFactorGraph allFactors3;
  allFactors3.push_back(newFactors2);
  allFactors3.push_back(newFactors3);
  Values allValues3;
  allValues3.insert(newValues2);
  allValues3.insert(newValues3);
  Values expected3 = BatchOptimize(allFactors3, allValues3);
  // Get actual linearization point
  Values actual3 = smoother.calculateEstimate();
  // Check
  CHECK(assert_equal(expected3, actual3, 1e-6));

  // Also check the single-variable version
  Pose3 expectedPose1 = expected3.at<Pose3>(1);
  Pose3 expectedPose2 = expected3.at<Pose3>(2);
  Pose3 expectedPose3 = expected3.at<Pose3>(3);
  Pose3 expectedPose4 = expected3.at<Pose3>(4);
  // Also check the single-variable version
  Pose3 actualPose1 = smoother.calculateEstimate<Pose3>(1);
  Pose3 actualPose2 = smoother.calculateEstimate<Pose3>(2);
  Pose3 actualPose3 = smoother.calculateEstimate<Pose3>(3);
  Pose3 actualPose4 = smoother.calculateEstimate<Pose3>(4);
  // Check
  CHECK(assert_equal(expectedPose1, actualPose1, 1e-6));
  CHECK(assert_equal(expectedPose2, actualPose2, 1e-6));
  CHECK(assert_equal(expectedPose3, actualPose3, 1e-6));
  CHECK(assert_equal(expectedPose4, actualPose4, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, update_empty )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

  // Call update
  smoother.update();
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, update_multiple )
{
  // Create a Concurrent Batch Smoother
  LevenbergMarquardtParams parameters;
  ConcurrentBatchSmoother smoother(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = smoother.calculateEstimate();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the smoother
  NonlinearFactorGraph newFactors2;
  newFactors2.addPrior(1, poseInitial, noisePrior);
  newFactors2.push_back(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
  Values newValues2;
  newValues2.insert(1, Pose3().compose(poseError));
  newValues2.insert(2, newValues2.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  smoother.update(newFactors2, newValues2);

  // Expected values from full batch optimization
  NonlinearFactorGraph allFactors2;
  allFactors2.push_back(newFactors2);
  Values allValues2;
  allValues2.insert(newValues2);
  Values expected2 = BatchOptimize(allFactors2, allValues2);
  // Get actual linearization point
  Values actual2 = smoother.calculateEstimate();
  // Check
  CHECK(assert_equal(expected2, actual2, 1e-6));

  // Add some more factors to the smoother
  NonlinearFactorGraph newFactors3;
  newFactors3.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors3.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  Values newValues3;
  newValues3.insert(3, newValues2.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  newValues3.insert(4, newValues3.at<Pose3>(3).compose(poseOdometry).compose(poseError));
  smoother.update(newFactors3, newValues3);

  // Expected values from full batch optimization
  NonlinearFactorGraph allFactors3;
  allFactors3.push_back(newFactors2);
  allFactors3.push_back(newFactors3);
  Values allValues3;
  allValues3.insert(newValues2);
  allValues3.insert(newValues3);
  Values expected3 = BatchOptimize(allFactors3, allValues3);
  // Get actual linearization point
  Values actual3 = smoother.calculateEstimate();
  // Check
  CHECK(assert_equal(expected3, actual3, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, synchronize_empty )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

  // Create empty containers *from* the filter
  NonlinearFactorGraph smootherFactors, filterSumarization;
  Values smootherValues, filterSeparatorValues;

  // Create expected values: these will be empty for this case
  NonlinearFactorGraph expectedSmootherSummarization;
  Values expectedSmootherSeparatorValues;

  // Synchronize
  NonlinearFactorGraph actualSmootherSummarization;
  Values actualSmootherSeparatorValues;
  smoother.presync();
  smoother.getSummarizedFactors(actualSmootherSummarization, actualSmootherSeparatorValues);
  smoother.synchronize(smootherFactors, smootherValues, filterSumarization, filterSeparatorValues);
  smoother.postsync();

  // Check
  CHECK(assert_equal(expectedSmootherSummarization, actualSmootherSummarization, 1e-6));
  CHECK(assert_equal(expectedSmootherSeparatorValues, actualSmootherSeparatorValues, 1e-6));
}

/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, synchronize_1 )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  parameters.maxIterations = 1;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

  // Create a simple separator *from* the filter
  NonlinearFactorGraph smootherFactors, filterSumarization;
  Values smootherValues, filterSeparatorValues;
  filterSeparatorValues.insert(1, Pose3().compose(poseError));
  filterSeparatorValues.insert(2, filterSeparatorValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  filterSumarization.push_back(LinearContainerFactor(PriorFactor<Pose3>(1, poseInitial, noisePrior).linearize(filterSeparatorValues), filterSeparatorValues));
  filterSumarization.push_back(LinearContainerFactor(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery).linearize(filterSeparatorValues), filterSeparatorValues));

  // Create expected values: the smoother output will be empty for this case
  NonlinearFactorGraph expectedSmootherSummarization;
  Values expectedSmootherSeparatorValues;

  NonlinearFactorGraph actualSmootherSummarization;
  Values actualSmootherSeparatorValues;
  smoother.presync();
  smoother.getSummarizedFactors(actualSmootherSummarization, actualSmootherSeparatorValues);
  smoother.synchronize(smootherFactors, smootherValues, filterSumarization, filterSeparatorValues);
  smoother.postsync();

  // Check
  CHECK(assert_equal(expectedSmootherSummarization, actualSmootherSummarization, 1e-6));
  CHECK(assert_equal(expectedSmootherSeparatorValues, actualSmootherSeparatorValues, 1e-6));


  // Update the smoother
  smoother.update();

  // Check the factor graph. It should contain only the filter-provided factors
  NonlinearFactorGraph expectedFactorGraph = filterSumarization;
  NonlinearFactorGraph actualFactorGraph = smoother.getFactors();
  CHECK(assert_equal(expectedFactorGraph, actualFactorGraph, 1e-6));

  // Check the optimized value of smoother state
  NonlinearFactorGraph allFactors;
  allFactors.push_back(filterSumarization);
  Values allValues;
  allValues.insert(filterSeparatorValues);
  Values expectedValues = BatchOptimize(allFactors, allValues, parameters.maxIterations);
  Values actualValues = smoother.calculateEstimate();
  CHECK(assert_equal(expectedValues, actualValues, 1e-6));

  // Check the linearization point. The separator should remain identical to the filter provided values
  Values expectedLinearizationPoint = filterSeparatorValues;
  Values actualLinearizationPoint = smoother.getLinearizationPoint();
  CHECK(assert_equal(expectedLinearizationPoint, actualLinearizationPoint, 1e-6));
}


/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, synchronize_2 )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  parameters.maxIterations = 1;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

  // Create a separator and cached smoother factors *from* the filter
  NonlinearFactorGraph smootherFactors, filterSumarization;
  Values smootherValues, filterSeparatorValues;
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  filterSeparatorValues.insert(1, Pose3().compose(poseError));
  filterSeparatorValues.insert(2, filterSeparatorValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  filterSumarization.push_back(LinearContainerFactor(PriorFactor<Pose3>(1, poseInitial, noisePrior).linearize(filterSeparatorValues), filterSeparatorValues));
  filterSumarization.push_back(LinearContainerFactor(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery).linearize(filterSeparatorValues), filterSeparatorValues));
  smootherFactors.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  smootherFactors.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  smootherValues.insert(3, filterSeparatorValues.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  smootherValues.insert(4, smootherValues.at<Pose3>(3).compose(poseOdometry).compose(poseError));

  // Create expected values: the smoother output will be empty for this case
  NonlinearFactorGraph expectedSmootherSummarization;
  Values expectedSmootherSeparatorValues;

  NonlinearFactorGraph actualSmootherSummarization;
  Values actualSmootherSeparatorValues;
  smoother.presync();
  smoother.getSummarizedFactors(actualSmootherSummarization, actualSmootherSeparatorValues);
  smoother.synchronize(smootherFactors, smootherValues, filterSumarization, filterSeparatorValues);
  smoother.postsync();

  // Check
  CHECK(assert_equal(expectedSmootherSummarization, actualSmootherSummarization, 1e-6));
  CHECK(assert_equal(expectedSmootherSeparatorValues, actualSmootherSeparatorValues, 1e-6));


  // Update the smoother
  smoother.update();

  // Check the factor graph. It should contain only the filter-provided factors
  NonlinearFactorGraph expectedFactorGraph;
  expectedFactorGraph.push_back(smootherFactors);
  expectedFactorGraph.push_back(filterSumarization);
  NonlinearFactorGraph actualFactorGraph = smoother.getFactors();
  CHECK(assert_equal(expectedFactorGraph, actualFactorGraph, 1e-6));

  // Check the optimized value of smoother state
  NonlinearFactorGraph allFactors;
  allFactors.push_back(filterSumarization);
  allFactors.push_back(smootherFactors);
  Values allValues;
  allValues.insert(filterSeparatorValues);
  allValues.insert(smootherValues);
  Values expectedValues = BatchOptimize(allFactors, allValues, parameters.maxIterations);
  Values actualValues = smoother.calculateEstimate();
  CHECK(assert_equal(expectedValues, actualValues, 1e-6));

  // Check the linearization point. The separator should remain identical to the filter provided values, but the others should be the optimal values
  Values expectedLinearizationPoint = BatchOptimize(allFactors, allValues, parameters.maxIterations);
  expectedLinearizationPoint.update(filterSeparatorValues);
  Values actualLinearizationPoint = smoother.getLinearizationPoint();
  CHECK(assert_equal(expectedLinearizationPoint, actualLinearizationPoint, 1e-6));
}



/* ************************************************************************* */
TEST( ConcurrentBatchSmoother, synchronize_3 )
{
  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  parameters.maxIterations = 1;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

  // Create a separator and cached smoother factors *from* the filter
  NonlinearFactorGraph smootherFactors, filterSumarization;
  Values smootherValues, filterSeparatorValues;
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  filterSeparatorValues.insert(1, Pose3().compose(poseError));
  filterSeparatorValues.insert(2, filterSeparatorValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  filterSumarization.push_back(LinearContainerFactor(PriorFactor<Pose3>(1, poseInitial, noisePrior).linearize(filterSeparatorValues), filterSeparatorValues));
  filterSumarization.push_back(LinearContainerFactor(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery).linearize(filterSeparatorValues), filterSeparatorValues));
  smootherFactors.push_back(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  smootherFactors.push_back(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  smootherFactors.addPrior(4, poseInitial, noisePrior);
  smootherValues.insert(3, filterSeparatorValues.at<Pose3>(2).compose(poseOdometry).compose(poseError));
  smootherValues.insert(4, smootherValues.at<Pose3>(3).compose(poseOdometry).compose(poseError));

  // Create expected values: the smoother output will be empty for this case
  NonlinearFactorGraph expectedSmootherSummarization;
  Values expectedSmootherSeparatorValues;

  NonlinearFactorGraph actualSmootherSummarization;
  Values actualSmootherSeparatorValues;
  smoother.presync();
  smoother.getSummarizedFactors(actualSmootherSummarization, actualSmootherSeparatorValues);
  smoother.synchronize(smootherFactors, smootherValues, filterSumarization, filterSeparatorValues);
  smoother.postsync();

  // Check
  CHECK(assert_equal(expectedSmootherSummarization, actualSmootherSummarization, 1e-6));
  CHECK(assert_equal(expectedSmootherSeparatorValues, actualSmootherSeparatorValues, 1e-6));


  // Update the smoother
  smoother.update();

  smoother.presync();
  smoother.getSummarizedFactors(actualSmootherSummarization, actualSmootherSeparatorValues);

  // Check the optimized value of smoother state
  NonlinearFactorGraph allFactors = smootherFactors;
  Values allValues = smoother.getLinearizationPoint();
  ordering = smoother.getOrdering();  // I'm really hoping this is an acceptable ordering...
  GaussianFactorGraph::shared_ptr linearFactors = allFactors.linearize(allValues);

  KeySet eliminateKeys = linearFactors->keys();
  for(const auto key: filterSeparatorValues.keys()) {
    eliminateKeys.erase(key);
  }
  KeyVector variables(eliminateKeys.begin(), eliminateKeys.end());
  GaussianFactorGraph result = *linearFactors->eliminatePartialMultifrontal(variables, EliminateCholesky).second;

  expectedSmootherSummarization.resize(0);
  for(const GaussianFactor::shared_ptr& factor: result) {
    expectedSmootherSummarization.push_back(LinearContainerFactor(factor, allValues));
  }

  CHECK(assert_equal(expectedSmootherSummarization, actualSmootherSummarization, 1e-6));

}

///* ************************************************************************* */
TEST( ConcurrentBatchSmoother, removeFactors_topology_1 )
{
  std::cout << "*********************** removeFactors_topology_1 ************************" << std::endl;

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

  // Add some factors to the smoother
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


  // Update the smoother: add all factors
  smoother.update(newFactors, newValues);

  // factor we want to remove
  // NOTE: we can remove factors, paying attention that the remaining graph remains connected
  // we remove a single factor, the number 1, which is a BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery);
  std::vector<size_t> removeFactorIndices(1,1);

  // Add no factors to the smoother (we only want to test the removal)
  NonlinearFactorGraph noFactors;
  Values noValues;
  smoother.update(noFactors, noValues, removeFactorIndices);

  NonlinearFactorGraph actualGraph = smoother.getFactors();

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
TEST( ConcurrentBatchSmoother, removeFactors_topology_2 )
{
  std::cout << "*********************** removeFactors_topology_2 ************************" << std::endl;
  // we try removing the last factor

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

  // Add some factors to the smoother
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

  // Update the smoother: add all factors
  smoother.update(newFactors, newValues);

  // factor we want to remove
  // NOTE: we can remove factors, paying attention that the remaining graph remains connected
  // we remove a single factor, the number 1, which is a BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery);
  std::vector<size_t> removeFactorIndices(1,4);

  // Add no factors to the smoother (we only want to test the removal)
  NonlinearFactorGraph noFactors;
  Values noValues;
  smoother.update(noFactors, noValues, removeFactorIndices);

  NonlinearFactorGraph actualGraph = smoother.getFactors();

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
TEST( ConcurrentBatchSmoother, removeFactors_topology_3 )
{
  std::cout << "*********************** removeFactors_topology_3 ************************" << std::endl;
  // we try removing the first factor

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  ConcurrentBatchSmoother Smoother(parameters);

  // Add some factors to the Smoother
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

  // Update the Smoother: add all factors
  Smoother.update(newFactors, newValues);

  // factor we want to remove
  // NOTE: we can remove factors, paying attention that the remaining graph remains connected
  // we remove a single factor, the number 0, which is a BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery);
  std::vector<size_t> removeFactorIndices(1,0);

  // Add no factors to the Smoother (we only want to test the removal)
  NonlinearFactorGraph noFactors;
  Values noValues;
  Smoother.update(noFactors, noValues, removeFactorIndices);

  NonlinearFactorGraph actualGraph = Smoother.getFactors();

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
TEST( ConcurrentBatchSmoother, removeFactors_values )
{
  std::cout << "*********************** removeFactors_values ************************" << std::endl;
  // we try removing the last factor

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;
  ConcurrentBatchSmoother Smoother(parameters);

  // Add some factors to the Smoother
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

  // Update the Smoother: add all factors
  Smoother.update(newFactors, newValues);

  // factor we want to remove
  // NOTE: we can remove factors, paying attention that the remaining graph remains connected
  // we remove a single factor, the number 4, which is a BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery);
  std::vector<size_t> removeFactorIndices(1,4);

  // Add no factors to the Smoother (we only want to test the removal)
  NonlinearFactorGraph noFactors;
  Values noValues;
  Smoother.update(noFactors, noValues, removeFactorIndices);

  NonlinearFactorGraph actualGraph = Smoother.getFactors();
  Values actualValues = Smoother.calculateEstimate();

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


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
