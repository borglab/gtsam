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
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam/geometry/Pose3.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;


// Set up initial pose, odometry difference, loop closure difference, and initialization errors
const Pose3 poseInitial;
const Pose3 poseOdometry( Rot3::RzRyRx(Vector_(3, 0.05, 0.10, -0.75)), Point3(1.0, -0.25, 0.10) );
const Pose3 poseError( Rot3::RzRyRx(Vector_(3, 0.01, 0.02, -0.1)), Point3(0.05, -0.05, 0.02) );

// Set up noise models for the factors
const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
const SharedDiagonal noiseOdometery = noiseModel::Diagonal::Sigmas(Vector_(6, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5));
const SharedDiagonal noiseLoop = noiseModel::Diagonal::Sigmas(Vector_(6, 0.25, 0.25, 0.25, 1.0, 1.0, 1.0));

// Create a derived class to allow testing protected member functions
class ConcurrentBatchFilterTester : public ConcurrentBatchFilter {
public:
  ConcurrentBatchFilterTester(const LevenbergMarquardtParams& parameters) : ConcurrentBatchFilter(parameters) { };
  virtual ~ConcurrentBatchFilterTester() { };

  // Add accessors to the protected members
  void presync() { ConcurrentBatchFilter::presync(); };
  void getSummarizedFactors(NonlinearFactorGraph& summarizedFactors, Values& rootValues) { ConcurrentBatchFilter::getSummarizedFactors(summarizedFactors, rootValues); };
  void getSmootherFactors(NonlinearFactorGraph& smootherFactors, Values& smootherValues) { ConcurrentBatchFilter::getSmootherFactors(smootherFactors, smootherValues); };
  void synchronize(const NonlinearFactorGraph& summarizedFactors, const Values& separatorValues) { ConcurrentBatchFilter::synchronize(summarizedFactors, separatorValues); };
  void postsync() { ConcurrentBatchFilter::postsync(); };
};

/* ************************************************************************* */
bool hessian_equal(const NonlinearFactorGraph& expected, const NonlinearFactorGraph& actual, const Values& theta, double tol = 1e-9) {

  // Verify the set of keys in both graphs are the same
  FastSet<Key> expectedKeys = expected.keys();
  FastSet<Key> actualKeys = actual.keys();
  bool keys_equal = (expectedKeys.size() == actualKeys.size()) && std::equal(expectedKeys.begin(), expectedKeys.end(), actualKeys.begin());
  if(!keys_equal) {
    std::cout << "Hessian Keys not equal:" << std::endl;

    std::cout << "Expected Keys: ";
    BOOST_FOREACH(Key key, expectedKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;

    std::cout << "Actual Keys: ";
    BOOST_FOREACH(Key key, actualKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;

    return false;
  }

  // Create an ordering
  Ordering ordering;
  BOOST_FOREACH(Key key, expectedKeys) {
    ordering.push_back(key);
  }

  // Linearize each factor graph
  GaussianFactorGraph expectedGaussian;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, expected) {
    if(factor)
      expectedGaussian.push_back( factor->linearize(theta, ordering) );
  }
  GaussianFactorGraph actualGaussian;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, actual) {
    if(factor)
      actualGaussian.push_back( factor->linearize(theta, ordering) );
  }

  // Convert linear factor graph into a dense Hessian
  Matrix expectedHessian = expectedGaussian.augmentedHessian();
  Matrix actualHessian = actualGaussian.augmentedHessian();

  // Zero out the lower-right entry. This corresponds to a constant in the optimization,
  // which does not affect the result. Further, in conversions between Jacobians and Hessians,
  // this term is ignored.
  expectedHessian(expectedHessian.rows()-1, expectedHessian.cols()-1) = 0.0;
  actualHessian(actualHessian.rows()-1, actualHessian.cols()-1) = 0.0;

  // Compare Hessians
  return assert_equal(expectedHessian, actualHessian, tol);
}

///* ************************************************************************* */
void CreateFactors(NonlinearFactorGraph& graph, Values& theta, size_t index1 = 0, size_t index2 = 1) {

  // Calculate all poses
  Pose3 poses[20];
  poses[0] = poseInitial;
  for(size_t index = 1; index < 20; ++index) {
    poses[index] = poses[index-1].compose(poseOdometry);
  }

  // Create all keys
  Key keys[20];
  for(size_t index = 0; index < 20; ++index) {
    keys[index] = Symbol('X', index);
  }

  // Create factors that will form a specific tree structure
  // Loop over the included timestamps
  for(size_t index = index1; index < index2; ++index) {

    switch(index) {
      case 0:
      {
        graph.add(PriorFactor<Pose3>(keys[0], poses[0], noisePrior));
        // Add new variables
        theta.insert(keys[0], poses[0].compose(poseError));
        break;
      }
      case 1:
      {
        // Add odometry factor between 0 and 1
        Pose3 poseDelta = poses[0].between(poses[1]);
        graph.add(BetweenFactor<Pose3>(keys[0], keys[1], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[1], poses[1].compose(poseError));
        break;
      }
      case 2:
      {
        break;
      }
      case 3:
      {
        // Add odometry factor between 1 and 3
        Pose3 poseDelta = poses[1].between(poses[3]);
        graph.add(BetweenFactor<Pose3>(keys[1], keys[3], poseDelta, noiseOdometery));
        // Add odometry factor between 2 and 3
        poseDelta = poses[2].between(poses[3]);
        graph.add(BetweenFactor<Pose3>(keys[2], keys[3], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[2], poses[2].compose(poseError));
        theta.insert(keys[3], poses[3].compose(poseError));
        break;
      }
      case 4:
      {
        break;
      }
      case 5:
      {
        // Add odometry factor between 3 and 5
        Pose3 poseDelta = poses[3].between(poses[5]);
        graph.add(BetweenFactor<Pose3>(keys[3], keys[5], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[5], poses[5].compose(poseError));
        break;
      }
      case 6:
      {
        // Add odometry factor between 3 and 6
        Pose3 poseDelta = poses[3].between(poses[6]);
        graph.add(BetweenFactor<Pose3>(keys[3], keys[6], poseDelta, noiseOdometery));
        // Add odometry factor between 5 and 6
        poseDelta = poses[5].between(poses[6]);
        graph.add(BetweenFactor<Pose3>(keys[5], keys[6], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[6], poses[6].compose(poseError));
        break;
      }
      case 7:
      {
        // Add odometry factor between 4 and 7
        Pose3 poseDelta = poses[4].between(poses[7]);
        graph.add(BetweenFactor<Pose3>(keys[4], keys[7], poseDelta, noiseOdometery));
        // Add odometry factor between 6 and 7
        poseDelta = poses[6].between(poses[7]);
        graph.add(BetweenFactor<Pose3>(keys[6], keys[7], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[4], poses[4].compose(poseError));
        theta.insert(keys[7], poses[7].compose(poseError));
        break;
      }
      case 8:
        break;

      case 9:
      {
        // Add odometry factor between 6 and 9
        Pose3 poseDelta = poses[6].between(poses[9]);
        graph.add(BetweenFactor<Pose3>(keys[6], keys[9], poseDelta, noiseOdometery));
        // Add odometry factor between 7 and 9
        poseDelta = poses[7].between(poses[9]);
        graph.add(BetweenFactor<Pose3>(keys[7], keys[9], poseDelta, noiseOdometery));
        // Add odometry factor between 8 and 9
        poseDelta = poses[8].between(poses[9]);
        graph.add(BetweenFactor<Pose3>(keys[8], keys[9], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[8], poses[8].compose(poseError));
        theta.insert(keys[9], poses[9].compose(poseError));
        break;
      }
      case 10:
      {
        // Add odometry factor between 9 and 10
        Pose3 poseDelta = poses[9].between(poses[10]);
        graph.add(BetweenFactor<Pose3>(keys[9], keys[10], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[10], poses[10].compose(poseError));
        break;
      }
      case 11:
      {
        // Add odometry factor between 10 and 11
        Pose3 poseDelta = poses[10].between(poses[11]);
        graph.add(BetweenFactor<Pose3>(keys[10], keys[11], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[11], poses[11].compose(poseError));
        break;
      }
      case 12:
      {
        // Add odometry factor between 7 and 12
        Pose3 poseDelta = poses[7].between(poses[12]);
        graph.add(BetweenFactor<Pose3>(keys[7], keys[12], poseDelta, noiseOdometery));
        // Add odometry factor between 9 and 12
        poseDelta = poses[9].between(poses[12]);
        graph.add(BetweenFactor<Pose3>(keys[9], keys[12], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[12], poses[12].compose(poseError));
        break;
      }





      case 13:
      {
        // Add odometry factor between 10 and 13
        Pose3 poseDelta = poses[10].between(poses[13]);
        graph.add(BetweenFactor<Pose3>(keys[10], keys[13], poseDelta, noiseOdometery));
        // Add odometry factor between 12 and 13
        poseDelta = poses[12].between(poses[13]);
        graph.add(BetweenFactor<Pose3>(keys[12], keys[13], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[13], poses[13].compose(poseError));
        break;
      }
      case 14:
      {
        // Add odometry factor between 11 and 14
        Pose3 poseDelta = poses[11].between(poses[14]);
        graph.add(BetweenFactor<Pose3>(keys[11], keys[14], poseDelta, noiseOdometery));
        // Add odometry factor between 13 and 14
        poseDelta = poses[13].between(poses[14]);
        graph.add(BetweenFactor<Pose3>(keys[13], keys[14], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[14], poses[14].compose(poseError));
        break;
      }
      case 15:
        break;

      case 16:
      {
        // Add odometry factor between 13 and 16
        Pose3 poseDelta = poses[13].between(poses[16]);
        graph.add(BetweenFactor<Pose3>(keys[13], keys[16], poseDelta, noiseOdometery));
        // Add odometry factor between 14 and 16
        poseDelta = poses[14].between(poses[16]);
        graph.add(BetweenFactor<Pose3>(keys[14], keys[16], poseDelta, noiseOdometery));
        // Add odometry factor between 15 and 16
        poseDelta = poses[15].between(poses[16]);
        graph.add(BetweenFactor<Pose3>(keys[15], keys[16], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[15], poses[15].compose(poseError));
        theta.insert(keys[16], poses[16].compose(poseError));
        break;
      }
      case 17:
      {
        // Add odometry factor between 16 and 17
        Pose3 poseDelta = poses[16].between(poses[17]);
        graph.add(BetweenFactor<Pose3>(keys[16], keys[17], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[17], poses[17].compose(poseError));
        break;
      }
      case 18:
      {
        // Add odometry factor between 17 and 18
        Pose3 poseDelta = poses[17].between(poses[18]);
        graph.add(BetweenFactor<Pose3>(keys[17], keys[18], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[18], poses[18].compose(poseError));
        break;
      }
      case 19:
      {
        // Add odometry factor between 14 and 19
        Pose3 poseDelta = poses[14].between(poses[19]);
        graph.add(BetweenFactor<Pose3>(keys[14], keys[19], poseDelta, noiseOdometery));
        // Add odometry factor between 16 and 19
        poseDelta = poses[16].between(poses[19]);
        graph.add(BetweenFactor<Pose3>(keys[16], keys[19], poseDelta, noiseOdometery));
        // Add new variables
        theta.insert(keys[19], poses[19].compose(poseError));
        break;
      }

    }
  }

  return;
}

/* ************************************************************************* */
Values BatchOptimize(const NonlinearFactorGraph& graph, const Values& theta, const Values& separatorValues = Values()) {

  // Create an L-M optimizer
  LevenbergMarquardtParams parameters;
  parameters.linearSolverType = SuccessiveLinearizationParams::MULTIFRONTAL_QR;

  LevenbergMarquardtOptimizer optimizer(graph, theta, parameters);

  // Use a custom optimization loop so the linearization points can be controlled
  double currentError;
  do {
    // Force variables associated with root keys to keep the same linearization point
    if(separatorValues.size() > 0) {
      // Put the old values of the root keys back into the optimizer state
      optimizer.state().values.update(separatorValues);
      // Update the error value with the new theta
      optimizer.state().error = graph.error(optimizer.state().values);
    }

    // Do next iteration
    currentError = optimizer.error();
    optimizer.iterate();

  } while(optimizer.iterations() < parameters.maxIterations &&
      !checkConvergence(parameters.relativeErrorTol, parameters.absoluteErrorTol,
          parameters.errorTol, currentError, optimizer.error(), parameters.verbosity));

  // return the final optimized values
  return optimizer.values();
}

/* ************************************************************************* */
NonlinearFactorGraph MarginalFactors(const NonlinearFactorGraph& factors, const Values& values, const std::set<Key>& remainingKeys) {

  // Create an ordering with the remaining variable last
  std::map<Key, int> constraints;
  BOOST_FOREACH(Key key, remainingKeys) {
    constraints[key] = 1;
  }
  Ordering ordering = *factors.orderingCOLAMDConstrained(values, constraints);

  // Convert the remaining keys into indices
  std::vector<Index> remainingIndices;
  BOOST_FOREACH(Key key, remainingKeys) {
    remainingIndices.push_back(ordering.at(key));
  }

  // Solve for the Gaussian marginal factors
  GaussianSequentialSolver gss(*factors.linearize(values, ordering), true);
  GaussianFactorGraph linearMarginals = *gss.jointFactorGraph(remainingIndices);

  // Convert to LinearContainFactors
  return LinearContainerFactor::convertLinearGraph(linearMarginals, ordering, values);
}

/* ************************************************************************* */
template<class CONTAINER>
void FindFactorsWithAny(const CONTAINER& keys, const NonlinearFactorGraph& sourceFactors, NonlinearFactorGraph& destinationFactors) {

  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, sourceFactors) {
    NonlinearFactor::const_iterator key = factor->begin();
    while((key != factor->end()) && (!std::binary_search(keys.begin(), keys.end(), *key))) {
      ++key;
    }
    if(key != factor->end()) {
      destinationFactors.push_back(factor);
    }
  }

}

/* ************************************************************************* */
template<class CONTAINER>
void FindFactorsWithOnly(const CONTAINER& keys, const NonlinearFactorGraph& sourceFactors, NonlinearFactorGraph& destinationFactors) {

  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, sourceFactors) {
    NonlinearFactor::const_iterator key = factor->begin();
    while((key != factor->end()) && (std::binary_search(keys.begin(), keys.end(), *key))) {
      ++key;
    }
    if(key == factor->end()) {
      destinationFactors.push_back(factor);
    }
  }

}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, update_batch )
{
  // Test the 'update' function of the ConcurrentBatchFilter in a nonlinear environment.
  // Thus, a full L-M optimization and the ConcurrentBatchFilter results should be identical
  // This tests adds all of the factors to the filter at once (i.e. batch)

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Create containers to keep the full graph
  Values fullTheta;
  NonlinearFactorGraph fullGraph;

  // Create all factors
  CreateFactors(fullGraph, fullTheta, 0, 20);

  // Optimize with Concurrent Batch Filter
  filter.update(fullGraph, fullTheta, boost::none);
  Values actual = filter.calculateEstimate();


  // Optimize with L-M
  Values expected = BatchOptimize(fullGraph, fullTheta);

  // Check smoother versus batch
  CHECK(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, update_batch_with_marginalization )
{
  // Test the 'update' function of the ConcurrentBatchFilter in a nonlinear environment.
  // Thus, a full L-M optimization and the ConcurrentBatchFilter results should be identical
  // This tests adds all of the factors to the filter at once (i.e. batch)

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Create containers to keep the full graph
  Values fullTheta;
  NonlinearFactorGraph fullGraph;

  // Create all factors
  CreateFactors(fullGraph, fullTheta, 0, 20);

  // Create the set of key to marginalize out
  FastList<Key> marginalizeKeys;
  for(size_t j = 0; j < 15; ++j) {
    marginalizeKeys.push_back(Symbol('X', j));
  }

  // Optimize with Concurrent Batch Filter
  filter.update(fullGraph, fullTheta, marginalizeKeys);
  Values actual = filter.calculateEstimate();


  // Optimize with L-M
  Values expected = BatchOptimize(fullGraph, fullTheta);
  // Remove the marginalized keys
  for(size_t j = 0; j < 15; ++j) {
    expected.erase(Symbol('X', j));
  }

  // Check smoother versus batch
  CHECK(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, update_incremental )
{
  // Test the 'update' function of the ConcurrentBatchFilter in a nonlinear environment.
  // Thus, a full L-M optimization and the ConcurrentBatchFilter results should be identical
  // This tests adds the factors to the filter as they are created (i.e. incrementally)

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Create containers to keep the full graph
  Values fullTheta;
  NonlinearFactorGraph fullGraph;

  // Add odometry from time 0 to time 10
  for(size_t i = 0; i < 20; ++i) {
    // Create containers to keep the new factors
    Values newTheta;
    NonlinearFactorGraph newGraph;

    // Create factors
    CreateFactors(newGraph, newTheta, i, i+1);

    // Add these entries to the filter
    filter.update(newGraph, newTheta, boost::none);
    Values actual = filter.calculateEstimate();

    // Add these entries to the full batch version
    fullGraph.push_back(newGraph);
    fullTheta.insert(newTheta);
    Values expected = BatchOptimize(fullGraph, fullTheta);
    fullTheta = expected;

    // Compare filter solution with full batch
    CHECK(assert_equal(expected, actual, 1e-4));
  }

}

/* ************************************************************************* */
TEST( ConcurrentBatchFilter, update_incremental_with_marginalization )
{
  // Test the 'update' function of the ConcurrentBatchFilter in a nonlinear environment.
  // Thus, a full L-M optimization and the ConcurrentBatchFilter results should be identical
  // This tests adds the factors to the filter as they are created (i.e. incrementally)

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilter filter(parameters);

  // Create containers to keep the full graph
  Values fullTheta;
  NonlinearFactorGraph fullGraph;

  // Add odometry from time 0 to time 10
  for(size_t i = 0; i < 20; ++i) {

    // Create containers to keep the new factors
    Values newTheta;
    NonlinearFactorGraph newGraph;

    // Create factors
    CreateFactors(newGraph, newTheta, i, i+1);

    // Create the set of factors to marginalize
    FastList<Key> marginalizeKeys;
    if(i >= 5) {
      marginalizeKeys.push_back(Symbol('X', i-5));
    }

    // Add these entries to the filter
    filter.update(newGraph, newTheta, marginalizeKeys);
    Values actual = filter.calculateEstimate();

    // Add these entries to the full batch version
    fullGraph.push_back(newGraph);
    fullTheta.insert(newTheta);
    Values expected = BatchOptimize(fullGraph, fullTheta);
    fullTheta = expected;
    // Remove marginalized keys
    for(int j = (int)i - 5; j >= 0; --j) {
      expected.erase(Symbol('X', j));
    }

    // Compare filter solution with full batch
    CHECK(assert_equal(expected, actual, 1e-4));
  }

}

/* ************************************************************************* */
TEST_UNSAFE( ConcurrentBatchFilter, synchronize )
{
  // Test the 'synchronize' function of the ConcurrentBatchFilter in a nonlinear environment.
  // The filter is operating on a known tree structure, so the factors and summarization can
  // be predicted for testing purposes

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Filter
  ConcurrentBatchFilterTester filter(parameters);

  // Create containers to keep the full graph
  Values newTheta, fullTheta;
  NonlinearFactorGraph newGraph, fullGraph;

  // Create factors from t=0 to t=12
  //  BAYES TREE:
  //    P( X7 X9 X12 )
  //      P( X10 | X9 )
  //        P( X11 | X10 )
  //      P( X8 | X9 )
  //      P( X6 | X7 X9 )
  //        P( X3 X5 | X6 )
  //          P( X2 | X3 )
  //          P( X1 | X3 )
  //            P( X0 | X1 )
  //      P( X4 | X7 )
  CreateFactors(newGraph, newTheta, 0, 13);
  fullTheta.insert(newTheta);
  fullGraph.push_back(newGraph);
  Values optimalTheta = BatchOptimize(fullGraph, fullTheta);

  // Optimize with Concurrent Batch Filter
  FastList<Key> marginalizeKeys;
  marginalizeKeys.push_back(Symbol('X',  0));
  marginalizeKeys.push_back(Symbol('X',  1));
  marginalizeKeys.push_back(Symbol('X',  2));
  marginalizeKeys.push_back(Symbol('X',  3));
  marginalizeKeys.push_back(Symbol('X',  4));
  marginalizeKeys.push_back(Symbol('X',  5));
  marginalizeKeys.push_back(Symbol('X',  6));
  filter.update(newGraph, newTheta, marginalizeKeys);

  // Extract the nonlinear factors that should be sent to the smoother
  NonlinearFactorGraph expectedSmootherFactors;
  FindFactorsWithAny(marginalizeKeys, fullGraph, expectedSmootherFactors);

  // Extract smoother values
  Values expectedSmootherValues;
  BOOST_FOREACH(Key key, marginalizeKeys) {
    expectedSmootherValues.insert(key, optimalTheta.at(key));
  }

  // Extract the filter summarized factors on the separator
  // ( as defined by the smoother branch separators {X7,X9} and {X7} )
  std::set<Key> separatorKeys;
  separatorKeys.insert(Symbol('X',  7));
  separatorKeys.insert(Symbol('X',  9));

  // Marginal factors remaining after marginalizing out all non-separator keys from the filter factors
  std::set<Key> filterKeys;
  filterKeys.insert(Symbol('X',  7));
  filterKeys.insert(Symbol('X',  8));
  filterKeys.insert(Symbol('X',  9));
  filterKeys.insert(Symbol('X', 10));
  filterKeys.insert(Symbol('X', 11));
  filterKeys.insert(Symbol('X', 12));
  NonlinearFactorGraph filterFactors;
  FindFactorsWithOnly(filterKeys, fullGraph, filterFactors);
  Values filterValues;
  BOOST_FOREACH(Key key, filterKeys) {
    filterValues.insert(key, optimalTheta.at(key));
  }
  NonlinearFactorGraph expectedFilterSumarization = MarginalFactors(filterFactors, filterValues, separatorKeys);

  // Extract the new separator values
  Values expectedSeparatorValues;
  BOOST_FOREACH(Key key, separatorKeys) {
    expectedSeparatorValues.insert(key, optimalTheta.at(key));
  }

  // Start the synchronization process
  NonlinearFactorGraph actualSmootherFactors, actualFilterSumarization, smootherSummarization;
  Values actualSmootherValues, actualSeparatorValues, smootherSeparatorValues;
  filter.presync();
  filter.synchronize(smootherSummarization, smootherSeparatorValues);  // Supplying an empty factor graph here
  filter.getSmootherFactors(actualSmootherFactors, actualSmootherValues);
  filter.getSummarizedFactors(actualFilterSumarization, actualSeparatorValues);
  filter.postsync();

  // Compare filter sync variables versus the expected
  CHECK(hessian_equal(expectedSmootherFactors, actualSmootherFactors, optimalTheta, 1e-8));
  CHECK(assert_equal(expectedSmootherValues, actualSmootherValues, 1e-4));
  CHECK(hessian_equal(expectedFilterSumarization, actualFilterSumarization, optimalTheta, 1e-9));
  CHECK(assert_equal(expectedSeparatorValues, actualSeparatorValues, 1e-4));





  // Calculate the quantities that would be coming from the smoother for the next iteration
  expectedSmootherValues.insert(expectedSeparatorValues);
  smootherSummarization = MarginalFactors(expectedSmootherFactors, expectedSmootherValues, separatorKeys);
  smootherSeparatorValues = expectedSeparatorValues;

  // Now add additional factors to the filter and re-sync
  //  BAYES TREE:
  //    P( X14 X16 X19 )
  //      P( X17 | X16 )
  //        P( X18 | X17 )
  //      P( X15 | X16 )
  //      P( X13 | X14 X16 )
  //        P( X11 | X13 X14 )
  //          P( X10 | X11 X13 )
  //            P( X12 | X10 X13 )
  //              P( X9 | X12 X10 )
  //                P( X7 | X9 X12 )
  //                  P( X6 | X7 X9 )
  //                    P( X3 X5 | X6 )
  //                      P( X2 | X3 )
  //                      P( X1 | X3 )
  //                        P( X0 | X1 )
  //                  P( X4 | X7 )
  //                P( X8 | X9 )
  newGraph.resize(0);  newTheta.clear();
  CreateFactors(newGraph, newTheta, 13, 20);
  fullTheta.insert(newTheta);
  fullGraph.push_back(newGraph);
  optimalTheta = BatchOptimize(fullGraph, fullTheta);

  // Optimize with Concurrent Batch Filter
  marginalizeKeys.clear();
  marginalizeKeys.push_back(Symbol('X',  7));
  marginalizeKeys.push_back(Symbol('X',  8));
  marginalizeKeys.push_back(Symbol('X',  9));
  marginalizeKeys.push_back(Symbol('X', 10));
  marginalizeKeys.push_back(Symbol('X', 11));
  marginalizeKeys.push_back(Symbol('X', 12));
  marginalizeKeys.push_back(Symbol('X', 13));
  filter.update(newGraph, newTheta, marginalizeKeys);

  // Extract the nonlinear factors that should be sent to the smoother
  NonlinearFactorGraph newSmootherFactors;
  FindFactorsWithAny(marginalizeKeys, fullGraph, newSmootherFactors);
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, expectedSmootherFactors) {
    NonlinearFactorGraph::iterator iter = std::find(newSmootherFactors.begin(), newSmootherFactors.end(), factor);
    if(iter != newSmootherFactors.end()) {
      newSmootherFactors.remove(iter - newSmootherFactors.begin());
    }
  }
  expectedSmootherFactors.resize(0);
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, newSmootherFactors) {
    if(factor) {
      expectedSmootherFactors.push_back(factor);
    }
  }

  // Extract smoother values
  expectedSmootherValues.clear();
  BOOST_FOREACH(Key key, marginalizeKeys) {
    expectedSmootherValues.insert(key, optimalTheta.at(key));
  }

  // Extract the filter summarized factors on the separator
  // ( as defined by the smoother branch separators {X14,X16} )
  separatorKeys.clear();
  separatorKeys.insert(Symbol('X', 14));
  separatorKeys.insert(Symbol('X', 16));

  // Marginal factors remaining after marginalizing out all non-separator keys from the filter factors
  filterKeys.clear();
  filterKeys.insert(Symbol('X', 14));
  filterKeys.insert(Symbol('X', 15));
  filterKeys.insert(Symbol('X', 16));
  filterKeys.insert(Symbol('X', 17));
  filterKeys.insert(Symbol('X', 18));
  filterKeys.insert(Symbol('X', 19));
  filterFactors.resize(0);
  FindFactorsWithOnly(filterKeys, fullGraph, filterFactors);
  filterValues.clear();
  BOOST_FOREACH(Key key, filterKeys) {
    filterValues.insert(key, optimalTheta.at(key));
  }
  expectedFilterSumarization = MarginalFactors(filterFactors, filterValues, separatorKeys);

  // Extract the new separator values
  expectedSeparatorValues.clear();
  BOOST_FOREACH(Key key, separatorKeys) {
    expectedSeparatorValues.insert(key, optimalTheta.at(key));
  }


  // Start the synchronization process
  actualSmootherFactors.resize(0); actualFilterSumarization.resize(0);
  actualSmootherValues.clear(); actualSeparatorValues.clear();
  smootherSeparatorValues = expectedSeparatorValues;
  filter.presync();
  filter.synchronize(smootherSummarization, smootherSeparatorValues);
  filter.getSmootherFactors(actualSmootherFactors, actualSmootherValues);
  filter.getSummarizedFactors(actualFilterSumarization, actualSeparatorValues);
  filter.postsync();



  // Compare filter sync variables versus the expected
  CHECK(hessian_equal(expectedSmootherFactors, actualSmootherFactors, optimalTheta, 1e-8));
  CHECK(assert_equal(expectedSmootherValues, actualSmootherValues, 1e-3));
  CHECK(hessian_equal(expectedFilterSumarization, actualFilterSumarization, optimalTheta, 1e-8));
  CHECK(assert_equal(expectedSeparatorValues, actualSeparatorValues, 1e-4));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
