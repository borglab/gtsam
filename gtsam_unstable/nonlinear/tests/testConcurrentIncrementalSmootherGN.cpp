/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testConcurrentIncrementalSmoother.cpp
 * @brief   Unit tests for the Concurrent Batch Smoother
 * @author  Stephen Williams (swilliams8@gatech.edu)
 * @date    Jan 5, 2013
 */

#include <gtsam_unstable/nonlinear/ConcurrentIncrementalSmoother.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

namespace {

// Set up initial pose, odometry difference, loop closure difference, and initialization errors
const Pose3 poseInitial;
const Pose3 poseOdometry( Rot3::RzRyRx(Vector_(3, 0.05, 0.10, -0.75)), Point3(1.0, -0.25, 0.10) );
const Pose3 poseError( Rot3::RzRyRx(Vector_(3, 0.01, 0.02, -0.1)), Point3(0.05, -0.05, 0.02) );

// Set up noise models for the factors
const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
const SharedDiagonal noiseOdometery = noiseModel::Diagonal::Sigmas(Vector_(6, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5));
const SharedDiagonal noiseLoop = noiseModel::Diagonal::Sigmas(Vector_(6, 0.25, 0.25, 0.25, 1.0, 1.0, 1.0));

/* ************************************************************************* */
Values BatchOptimize(const NonlinearFactorGraph& graph, const Values& theta, int maxIter = 100) {

  // Create an L-M optimizer
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
//  parameters.maxIterations = maxIter;
//  parameters.lambdaUpperBound = 1;
//  parameters.lambdaInitial = 1;
//  parameters.verbosity = NonlinearOptimizerParams::ERROR;
//  parameters.verbosityLM = ISAM2Params::DAMPED;
//  parameters.linearSolverType = SuccessiveLinearizationParams::MULTIFRONTAL_QR;

  ISAM2 optimizer(parameters);
  optimizer.update( graph, theta );
  Values result = optimizer.calculateEstimate();
  return result;

}

} // end namespace




/* ************************************************************************* */
TEST( ConcurrentIncrementalSmootherGN, equals )
{
  // TODO: Test 'equals' more vigorously

  // Create a Concurrent Batch Smoother
  ISAM2Params parameters1;
  parameters1.optimizationParams = ISAM2GaussNewtonParams();
  ConcurrentIncrementalSmoother smoother1(parameters1);

  // Create an identical Concurrent Batch Smoother
  ISAM2Params parameters2;
  parameters2.optimizationParams = ISAM2GaussNewtonParams();
  ConcurrentIncrementalSmoother smoother2(parameters2);

  // Create a different Concurrent Batch Smoother
  ISAM2Params parameters3;
  parameters3.optimizationParams = ISAM2GaussNewtonParams();
//  parameters3.maxIterations = 1;
  ConcurrentIncrementalSmoother smoother3(parameters3);

  CHECK(assert_equal(smoother1, smoother1));
  CHECK(assert_equal(smoother1, smoother2));
//  CHECK(assert_inequal(smoother1, smoother3));
}

/* ************************************************************************* */
TEST( ConcurrentIncrementalSmootherGN, getFactors )
{
  // Create a Concurrent Batch Smoother
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
  ConcurrentIncrementalSmoother smoother(parameters);

  // Expected graph is empty
  NonlinearFactorGraph expected1;
  // Get actual graph
  NonlinearFactorGraph actual1 = smoother.getFactors();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the smoother
  NonlinearFactorGraph newFactors1;
  newFactors1.add(PriorFactor<Pose3>(1, poseInitial, noisePrior));
  newFactors1.add(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
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
  newFactors2.add(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors2.add(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
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
TEST( ConcurrentIncrementalSmootherGN, getLinearizationPoint )
{
  // Create a Concurrent Batch Smoother
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
  ConcurrentIncrementalSmoother smoother(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = smoother.getLinearizationPoint();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the smoother
  NonlinearFactorGraph newFactors1;
  newFactors1.add(PriorFactor<Pose3>(1, poseInitial, noisePrior));
  newFactors1.add(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
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
  newFactors2.add(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors2.add(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
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
TEST( ConcurrentIncrementalSmootherGN, getOrdering )
{
  // TODO: Think about how to check ordering...
}

/* ************************************************************************* */
TEST( ConcurrentIncrementalSmootherGN, getDelta )
{
  // TODO: Think about how to check ordering...
}

/* ************************************************************************* */
TEST( ConcurrentIncrementalSmootherGN, calculateEstimate )
{
  // Create a Concurrent Batch Smoother
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
  ConcurrentIncrementalSmoother smoother(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = smoother.calculateEstimate();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the smoother
  NonlinearFactorGraph newFactors2;
  newFactors2.add(PriorFactor<Pose3>(1, poseInitial, noisePrior));
  newFactors2.add(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
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
  newFactors3.add(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors3.add(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
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
TEST( ConcurrentIncrementalSmootherGN, update_empty )
{
  // Create a set of optimizer parameters
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
  // Create a Concurrent Batch Smoother
  ConcurrentIncrementalSmoother smoother(parameters);

  // Call update
  smoother.update();
}

/* ************************************************************************* */
TEST( ConcurrentIncrementalSmootherGN, update_multiple )
{
  // Create a Concurrent Batch Smoother
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
  ConcurrentIncrementalSmoother smoother(parameters);

  // Expected values is empty
  Values expected1;
  // Get Linearization Point
  Values actual1 = smoother.calculateEstimate();
  // Check
  CHECK(assert_equal(expected1, actual1));

  // Add some factors to the smoother
  NonlinearFactorGraph newFactors2;
  newFactors2.add(PriorFactor<Pose3>(1, poseInitial, noisePrior));
  newFactors2.add(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery));
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
  newFactors3.add(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  newFactors3.add(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
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
TEST( ConcurrentIncrementalSmootherGN, synchronize_empty )
{
  // Create a set of optimizer parameters
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
  // Create a Concurrent Batch Smoother
  ConcurrentIncrementalSmoother smoother(parameters);

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
TEST( ConcurrentIncrementalSmootherGN, synchronize_1 )
{
  // Create a set of optimizer parameters
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
//  parameters.maxIterations = 1;

  // Create a Concurrent Batch Smoother
  ConcurrentIncrementalSmoother smoother(parameters);

  // Create a simple separator *from* the filter
  NonlinearFactorGraph smootherFactors, filterSumarization;
  Values smootherValues, filterSeparatorValues;
  filterSeparatorValues.insert(1, Pose3().compose(poseError));
  filterSeparatorValues.insert(2, filterSeparatorValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  filterSumarization.add(LinearContainerFactor(PriorFactor<Pose3>(1, poseInitial, noisePrior).linearize(filterSeparatorValues, ordering), ordering, filterSeparatorValues));
  filterSumarization.add(LinearContainerFactor(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery).linearize(filterSeparatorValues, ordering), ordering, filterSeparatorValues));

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
  Values expectedValues = BatchOptimize(allFactors, allValues,1);
  Values actualValues = smoother.calculateEstimate();
  CHECK(assert_equal(expectedValues, actualValues, 1e-6));

  // Check the linearization point. The separator should remain identical to the filter provided values
  Values expectedLinearizationPoint = filterSeparatorValues;
  Values actualLinearizationPoint = smoother.getLinearizationPoint();
  CHECK(assert_equal(expectedLinearizationPoint, actualLinearizationPoint, 1e-6));
}


/* ************************************************************************* */
TEST( ConcurrentIncrementalSmootherGN, synchronize_2 )
{
  // Create a set of optimizer parameters
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
//  parameters.maxIterations = 1;
//  parameters.lambdaUpperBound = 1;
//  parameters.lambdaInitial = 1;
//  parameters.verbosity = NonlinearOptimizerParams::ERROR;
//  parameters.verbosityLM = ISAM2Params::DAMPED;

  // Create a Concurrent Batch Smoother
  ConcurrentIncrementalSmoother smoother(parameters);

  // Create a separator and cached smoother factors *from* the filter
  NonlinearFactorGraph smootherFactors, filterSumarization;
  Values smootherValues, filterSeparatorValues;
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  filterSeparatorValues.insert(1, Pose3().compose(poseError));
  filterSeparatorValues.insert(2, filterSeparatorValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  filterSumarization.add(LinearContainerFactor(PriorFactor<Pose3>(1, poseInitial, noisePrior).linearize(filterSeparatorValues, ordering), ordering, filterSeparatorValues));
  filterSumarization.add(LinearContainerFactor(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery).linearize(filterSeparatorValues, ordering), ordering, filterSeparatorValues));
  smootherFactors.add(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  smootherFactors.add(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
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
//  allValues.print("Batch LinPoint:\n");
  Values expectedValues = BatchOptimize(allFactors, allValues, 1);
  Values actualValues = smoother.calculateEstimate();
  CHECK(assert_equal(expectedValues, actualValues, 1e-6));

  // Check the linearization point. The separator should remain identical to the filter provided values, but the others should be the optimal values
  // Isam2 is changing internally the linearization points, so the following check is done only on the separator variables
//  Values expectedLinearizationPoint = BatchOptimize(allFactors, allValues, 1);
  Values expectedLinearizationPoint = filterSeparatorValues;
  Values actualLinearizationPoint;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, filterSeparatorValues) {
    actualLinearizationPoint.insert(key_value.key, smoother.getLinearizationPoint().at(key_value.key));
  }
  CHECK(assert_equal(expectedLinearizationPoint, actualLinearizationPoint, 1e-6));
}



/* ************************************************************************* */
TEST( ConcurrentIncrementalSmootherGN, synchronize_3 )
{
  // Create a set of optimizer parameters
  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
//  parameters.maxIterations = 1;
//  parameters.lambdaUpperBound = 1;
//  parameters.lambdaInitial = 1;
//  parameters.verbosity = NonlinearOptimizerParams::ERROR;
//  parameters.verbosityLM = ISAM2Params::DAMPED;

  // Create a Concurrent Batch Smoother
  ConcurrentIncrementalSmoother smoother(parameters);

  // Create a separator and cached smoother factors *from* the filter
  NonlinearFactorGraph smootherFactors, filterSumarization;
  Values smootherValues, filterSeparatorValues;
  Ordering ordering;
  ordering.push_back(1);
  ordering.push_back(2);
  filterSeparatorValues.insert(1, Pose3().compose(poseError));
  filterSeparatorValues.insert(2, filterSeparatorValues.at<Pose3>(1).compose(poseOdometry).compose(poseError));
  filterSumarization.add(LinearContainerFactor(PriorFactor<Pose3>(1, poseInitial, noisePrior).linearize(filterSeparatorValues, ordering), ordering, filterSeparatorValues));
  filterSumarization.add(LinearContainerFactor(BetweenFactor<Pose3>(1, 2, poseOdometry, noiseOdometery).linearize(filterSeparatorValues, ordering), ordering, filterSeparatorValues));
  smootherFactors.add(BetweenFactor<Pose3>(2, 3, poseOdometry, noiseOdometery));
  smootherFactors.add(BetweenFactor<Pose3>(3, 4, poseOdometry, noiseOdometery));
  smootherFactors.add(PriorFactor<Pose3>(4, poseInitial, noisePrior));
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

  GaussianFactorGraph::shared_ptr LinFactorGraph = allFactors.linearize(allValues, ordering);
//  GaussianSequentialSolver GSS = GaussianSequentialSolver(*LinFactorGraph);
//  GaussianBayesNet::shared_ptr GBNsptr = GSS.eliminate();

  FastSet<Index> allkeys = LinFactorGraph->keys();
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, filterSeparatorValues) {
    Index index = ordering.at(key_value.key);
    allkeys.erase(index);
  }
  std::vector<Index> variables(allkeys.begin(), allkeys.end());
  std::pair<GaussianConditional::shared_ptr, GaussianFactorGraph> result = LinFactorGraph->eliminate(variables, EliminateCholesky);

  expectedSmootherSummarization.resize(0);
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, result.second) {
    expectedSmootherSummarization.add(LinearContainerFactor(factor, ordering, allValues));
  }

  CHECK(assert_equal(expectedSmootherSummarization, actualSmootherSummarization, 1e-6));

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
