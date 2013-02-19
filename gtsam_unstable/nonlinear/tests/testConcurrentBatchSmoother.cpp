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
#include <gtsam_unstable/nonlinear/LinearizedFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Key.h>
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
class ConcurrentBatchSmootherTester : public ConcurrentBatchSmoother {
public:
  ConcurrentBatchSmootherTester(const LevenbergMarquardtParams& parameters) : ConcurrentBatchSmoother(parameters) { };
  virtual ~ConcurrentBatchSmootherTester() { };

  // Add accessors to the protected members
  void presync() {
    ConcurrentBatchSmoother::presync();
  };
  void getSummarizedFactors(NonlinearFactorGraph& summarizedFactors) {
    ConcurrentBatchSmoother::getSummarizedFactors(summarizedFactors);
  };
  void synchronize(const NonlinearFactorGraph& smootherFactors, const Values& smootherValues, const NonlinearFactorGraph& summarizedFactors, const Values& rootValues) {
    ConcurrentBatchSmoother::synchronize(smootherFactors, smootherValues, summarizedFactors, rootValues);
  };
  void postsync() {
    ConcurrentBatchSmoother::postsync();
  };
};

/* ************************************************************************* */
bool hessian_equal(const NonlinearFactorGraph& expected, const NonlinearFactorGraph& actual, const Values& theta, double tol = 1e-9) {

  FastSet<Key> expectedKeys = expected.keys();
  FastSet<Key> actualKeys = actual.keys();

  // Verify the set of keys in both graphs are the same
  if(!std::equal(expectedKeys.begin(), expectedKeys.end(), actualKeys.begin()))
    return false;

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
Values BatchOptimize(const NonlinearFactorGraph& graph, const Values& theta, const Values& rootValues = Values()) {

  // Create an L-M optimizer
  LevenbergMarquardtParams parameters;
  parameters.linearSolverType = SuccessiveLinearizationParams::MULTIFRONTAL_QR;

  LevenbergMarquardtOptimizer optimizer(graph, theta, parameters);

  // Use a custom optimization loop so the linearization points can be controlled
  double currentError;
  do {
    // Force variables associated with root keys to keep the same linearization point
    if(rootValues.size() > 0) {
      // Put the old values of the root keys back into the optimizer state
      optimizer.state().values.update(rootValues);
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
void FindFactorsWithAny(const std::set<Key>& keys, const NonlinearFactorGraph& sourceFactors, NonlinearFactorGraph& destinationFactors) {

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
void FindFactorsWithOnly(const std::set<Key>& keys, const NonlinearFactorGraph& sourceFactors, NonlinearFactorGraph& destinationFactors) {

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
typedef BayesTree<GaussianConditional,ISAM2Clique>::sharedClique Clique;
void SymbolicPrintTree(const Clique& clique, const Ordering& ordering, const std::string indent = "") {
  std::cout << indent << "P( ";
  BOOST_FOREACH(Index index, clique->conditional()->frontals()){
    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
  }
  if(clique->conditional()->nrParents() > 0) {
    std::cout << "| ";
  }
  BOOST_FOREACH(Index index, clique->conditional()->parents()){
    std::cout << DefaultKeyFormatter(ordering.key(index)) << " ";
  }
  std::cout << ")" << std::endl;

  BOOST_FOREACH(const Clique& child, clique->children()) {
    SymbolicPrintTree(child, ordering, indent+"  ");
  }
}

/* ************************************************************************* */
TEST_UNSAFE( ConcurrentBatchSmoother, update_Batch )
{
  // Test the 'update' function of the ConcurrentBatchSmoother in a nonlinear environment.
  // Thus, a full L-M optimization and the ConcurrentBatchSmoother results should be identical
  // This tests adds all of the factors to the smoother at once (i.e. batch)

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

  // Create containers to keep the full graph
  Values fullTheta;
  NonlinearFactorGraph fullGraph;

  // Create all factors
  CreateFactors(fullGraph, fullTheta, 0, 20);

  // Optimize with Concurrent Batch Smoother
  smoother.update(fullGraph, fullTheta);
  Values actual = smoother.calculateEstimate();

  // Optimize with L-M
  Values expected = BatchOptimize(fullGraph, fullTheta);

  // Check smoother versus batch
  CHECK(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST_UNSAFE( ConcurrentBatchSmoother, update_Incremental )
{
  // Test the 'update' function of the ConcurrentBatchSmoother in a nonlinear environment.
  // Thus, a full L-M optimization and the ConcurrentBatchSmoother results should be identical
  // This tests adds the factors to the smoother as they are created (i.e. incrementally)

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmoother smoother(parameters);

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
    smoother.update(newGraph, newTheta);
    Values actual = smoother.calculateEstimate();

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
TEST_UNSAFE( ConcurrentBatchSmoother, synchronize )
{
  // Test the 'synchronize' function of the ConcurrentBatchSmoother in a nonlinear environment.
  // The smoother is operating on a known tree structure, so the factors and summarization can
  // be predicted for testing purposes

  // Create a set of optimizer parameters
  LevenbergMarquardtParams parameters;

  // Create a Concurrent Batch Smoother
  ConcurrentBatchSmootherTester smoother(parameters);

  // Create containers to keep the full graph
  Values fullTheta;
  NonlinearFactorGraph fullGraph;

  // Create factors for times 0 - 12
  // When eliminated with ordering (X2 X0 X1 X4 X5 X3 X6 X8 X11 X10 X7 X9 X12)augmentedHessian
  // ... this Bayes Tree is produced:
  // Bayes Tree:
  //   P( X7 X9 X12 )
  //     P( X10 | X9 )
  //       P( X11 | X10 )
  //     P( X8 | X9 )
  //     P( X6 | X7 X9 )
  //       P( X5 X3 | X6 )
  //         P( X1 | X3 )
  //           P( X0 | X1 )
  //         P( X2 | X3 )
  //     P( X4 | X7 )
  // We then produce the inputs necessary for the 'synchronize' function.
  // The smoother is branches X4 and X6, the filter is branches X8 and X10, and the root is (X7 X9 X12)
  CreateFactors(fullGraph, fullTheta, 0, 13);

  // Optimize the full graph
  Values optimalTheta = BatchOptimize(fullGraph, fullTheta);

  // Re-eliminate to create the Bayes Tree
  Ordering ordering;
  ordering.push_back(Symbol('X',  2));
  ordering.push_back(Symbol('X',  0));
  ordering.push_back(Symbol('X',  1));
  ordering.push_back(Symbol('X',  4));
  ordering.push_back(Symbol('X',  5));
  ordering.push_back(Symbol('X',  3));
  ordering.push_back(Symbol('X',  6));
  ordering.push_back(Symbol('X',  8));
  ordering.push_back(Symbol('X', 11));
  ordering.push_back(Symbol('X', 10));
  ordering.push_back(Symbol('X',  7));
  ordering.push_back(Symbol('X',  9));
  ordering.push_back(Symbol('X', 12));
  Values linpoint;
  linpoint.insert(optimalTheta);
  GaussianFactorGraph linearGraph = *fullGraph.linearize(linpoint, ordering);
  JunctionTree<GaussianFactorGraph, ISAM2Clique> jt(linearGraph);
  ISAM2Clique::shared_ptr root = jt.eliminate(EliminateQR);
  BayesTree<GaussianConditional, ISAM2Clique> bayesTree;
  bayesTree.insert(root);

  // Extract the values for the smoother keys. This consists of the branches: X4 and X6
  // Extract the non-root values from the initial values to test the smoother optimization
  Values smootherValues;
  smootherValues.insert(Symbol('X',  0), fullTheta.at(Symbol('X',  0)));
  smootherValues.insert(Symbol('X',  1), fullTheta.at(Symbol('X',  1)));
  smootherValues.insert(Symbol('X',  2), fullTheta.at(Symbol('X',  2)));
  smootherValues.insert(Symbol('X',  3), fullTheta.at(Symbol('X',  3)));
  smootherValues.insert(Symbol('X',  4), fullTheta.at(Symbol('X',  4)));
  smootherValues.insert(Symbol('X',  5), fullTheta.at(Symbol('X',  5)));
  smootherValues.insert(Symbol('X',  6), fullTheta.at(Symbol('X',  6)));

  // Extract the optimal root values
  Values rootValues;
  rootValues.insert(Symbol('X',  7), optimalTheta.at(Symbol('X',  7)));
  rootValues.insert(Symbol('X',  9), optimalTheta.at(Symbol('X',  9)));
  rootValues.insert(Symbol('X', 12), optimalTheta.at(Symbol('X', 12)));

  // Extract the nonlinear smoother factors as any factor with a non-root smoother key
  std::set<Key> smootherKeys;
  smootherKeys.insert(Symbol('X', 0));
  smootherKeys.insert(Symbol('X', 1));
  smootherKeys.insert(Symbol('X', 2));
  smootherKeys.insert(Symbol('X', 3));
  smootherKeys.insert(Symbol('X', 4));
  smootherKeys.insert(Symbol('X', 5));
  smootherKeys.insert(Symbol('X', 6));
  NonlinearFactorGraph smootherFactors;
  FindFactorsWithAny(smootherKeys, fullGraph, smootherFactors);

  // Extract the filter summarized factors. This consists of the linear cached factors from
  // the filter branches X8 and X10, as well as any nonlinear factor that involves only root keys
  NonlinearFactorGraph filterSummarization;
  filterSummarization.add(LinearizedJacobianFactor(boost::static_pointer_cast<JacobianFactor>(bayesTree.nodes().at(ordering.at(Symbol('X',  8)))->cachedFactor()), ordering, linpoint));
  filterSummarization.add(LinearizedJacobianFactor(boost::static_pointer_cast<JacobianFactor>(bayesTree.nodes().at(ordering.at(Symbol('X', 10)))->cachedFactor()), ordering, linpoint));
  std::set<Key> rootKeys;
  rootKeys.insert(Symbol('X',  7));
  rootKeys.insert(Symbol('X',  9));
  rootKeys.insert(Symbol('X', 12));
  FindFactorsWithOnly(rootKeys, fullGraph, filterSummarization);



  // Perform the synchronization procedure
  NonlinearFactorGraph actualSmootherSummarization;
  smoother.presync();
  smoother.getSummarizedFactors(actualSmootherSummarization);
  smoother.synchronize(smootherFactors, smootherValues, filterSummarization, rootValues);
  smoother.postsync();

  // Verify the returned smoother values is empty in the first iteration
  NonlinearFactorGraph expectedSmootherSummarization;
  CHECK(assert_equal(expectedSmootherSummarization, actualSmootherSummarization, 1e-4));



  // Perform a full update of the smoother. Since the root values/summarized filter factors were
  // created at the optimal values, the smoother should be identical to the batch optimization
  smoother.update();
  Values actualSmootherTheta = smoother.calculateEstimate();

  // Create the expected values as the optimal set
  Values expectedSmootherTheta;
  expectedSmootherTheta.insert(Symbol('X',  0), optimalTheta.at(Symbol('X',  0)));
  expectedSmootherTheta.insert(Symbol('X',  1), optimalTheta.at(Symbol('X',  1)));
  expectedSmootherTheta.insert(Symbol('X',  2), optimalTheta.at(Symbol('X',  2)));
  expectedSmootherTheta.insert(Symbol('X',  3), optimalTheta.at(Symbol('X',  3)));
  expectedSmootherTheta.insert(Symbol('X',  4), optimalTheta.at(Symbol('X',  4)));
  expectedSmootherTheta.insert(Symbol('X',  5), optimalTheta.at(Symbol('X',  5)));
  expectedSmootherTheta.insert(Symbol('X',  6), optimalTheta.at(Symbol('X',  6)));

  // Compare filter solution with full batch
  CHECK(assert_equal(expectedSmootherTheta, actualSmootherTheta, 1e-4));



  // Add a loop closure factor to the smoother and re-check. Since the filter
  // factors were created at the optimal linpoint, and since the new loop closure
  // does not involve filter keys, the smoother should still yeild the optimal solution
  // The new Bayes Tree is:
  // Bayes Tree:
  //   P( X7 X9 X12 )
  //     P( X10 | X9 )
  //       P( X11 | X10 )
  //     P( X8 | X9 )
  //     P( X6 | X7 X9 )
  //       P( X4 | X6 X7 )
  //         P( X3 X5 | X4 X6 )
  //           P( X2 | X3 )
  //           P( X1 | X3 X4 )
  //             P( X0 | X1 )
  Pose3 poseDelta = fullTheta.at<Pose3>(Symbol('X', 1)).between(fullTheta.at<Pose3>(Symbol('X', 4)));
  NonlinearFactor::shared_ptr loopClosure = NonlinearFactor::shared_ptr(new BetweenFactor<Pose3>(Symbol('X', 1), Symbol('X', 4), poseDelta, noiseOdometery));
  fullGraph.push_back(loopClosure);
  optimalTheta = BatchOptimize(fullGraph, fullTheta, rootValues);

  // Recreate the Bayes Tree
  linpoint.clear();
  linpoint.insert(optimalTheta);
  linpoint.update(rootValues);
  linearGraph = *fullGraph.linearize(linpoint, ordering);
  jt = JunctionTree<GaussianFactorGraph, ISAM2Clique>(linearGraph);
  root = jt.eliminate(EliminateQR);
  bayesTree = BayesTree<GaussianConditional, ISAM2Clique>();
  bayesTree.insert(root);

  // Add the loop closure to the smoother
  NonlinearFactorGraph newFactors;
  newFactors.push_back(loopClosure);
  smoother.update(newFactors);
  actualSmootherTheta = smoother.calculateEstimate();

  // Create the expected values as the optimal set
  expectedSmootherTheta.clear();
  expectedSmootherTheta.insert(Symbol('X',  0), optimalTheta.at(Symbol('X',  0)));
  expectedSmootherTheta.insert(Symbol('X',  1), optimalTheta.at(Symbol('X',  1)));
  expectedSmootherTheta.insert(Symbol('X',  2), optimalTheta.at(Symbol('X',  2)));
  expectedSmootherTheta.insert(Symbol('X',  3), optimalTheta.at(Symbol('X',  3)));
  expectedSmootherTheta.insert(Symbol('X',  4), optimalTheta.at(Symbol('X',  4)));
  expectedSmootherTheta.insert(Symbol('X',  5), optimalTheta.at(Symbol('X',  5)));
  expectedSmootherTheta.insert(Symbol('X',  6), optimalTheta.at(Symbol('X',  6)));

  // Compare filter solution with full batch
  // TODO: Check This
//  CHECK(assert_equal(expectedSmootherTheta, actualSmootherTheta, 1e-4));



  // Now perform a second synchronization to test the smoother-calculated summarization
  actualSmootherSummarization.resize(0);
  smootherFactors.resize(0);
  smootherValues.clear();
  smoother.presync();
  smoother.getSummarizedFactors(actualSmootherSummarization);
  smoother.synchronize(smootherFactors, smootherValues, filterSummarization, rootValues);
  smoother.postsync();

  // Extract the expected smoother summarization from the Bayes Tree
  // The smoother branches after the addition of the loop closure is only X6
  expectedSmootherSummarization.resize(0);
  JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(bayesTree.nodes().at(ordering.at(Symbol('X',  6)))->cachedFactor());
  LinearizedJacobianFactor::shared_ptr ljf(new LinearizedJacobianFactor(jf, ordering, linpoint));
  expectedSmootherSummarization.push_back(ljf);

  // Compare smoother factors with the expected factors by computing the hessian information matrix
  // TODO: Check This
//  CHECK(hessian_equal(expectedSmootherSummarization, actualSmootherSummarization, linpoint, 1e-4));



  // TODO: Modify the second synchronization so that the filter sends an additional set of factors.
  // I'm not sure what additional code this will exercise, but just for good measure.

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
