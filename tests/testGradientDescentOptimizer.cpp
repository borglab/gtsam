/**
 * @file   testGradientDescentOptimizer.cpp
 * @brief  
 * @author ydjian
 * @date   Jun 11, 2012
 */

#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>


using namespace std;
using namespace gtsam;


boost::tuple<pose2SLAM::Graph, Values> generateProblem() {

  // 1. Create graph container and add factors to it
  pose2SLAM::Graph graph ;

  // 2a. Add Gaussian prior
  Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
  SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1));
  graph.addPosePrior(1, priorMean, priorNoise);

  // 2b. Add odometry factors
  SharedDiagonal odometryNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
  graph.addRelativePose(1, 2, Pose2(2.0, 0.0, 0.0   ), odometryNoise);
  graph.addRelativePose(2, 3, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.addRelativePose(3, 4, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.addRelativePose(4, 5, Pose2(2.0, 0.0, M_PI_2), odometryNoise);

  // 2c. Add pose constraint
  SharedDiagonal constraintUncertainty = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
  graph.addRelativePose(5, 2, Pose2(2.0, 0.0, M_PI_2), constraintUncertainty);

  // 3. Create the data structure to hold the initialEstimate estinmate to the solution
  pose2SLAM::Values initialEstimate;
  Pose2 x1(0.5, 0.0, 0.2   ); initialEstimate.insertPose(1, x1);
  Pose2 x2(2.3, 0.1,-0.2   ); initialEstimate.insertPose(2, x2);
  Pose2 x3(4.1, 0.1, M_PI_2); initialEstimate.insertPose(3, x3);
  Pose2 x4(4.0, 2.0, M_PI  ); initialEstimate.insertPose(4, x4);
  Pose2 x5(2.1, 2.1,-M_PI_2); initialEstimate.insertPose(5, x5);

  return boost::tie(graph, initialEstimate);
}

/* ************************************************************************* */
TEST(optimize, ConjugateGradientOptimizer) {

  pose2SLAM::Graph graph ;
  pose2SLAM::Values initialEstimate;

  boost::tie(graph, initialEstimate) = generateProblem();
//  cout << "initial error = " << graph.error(initialEstimate) << endl ;

  // Single Step Optimization using Levenberg-Marquardt
  NonlinearOptimizerParams param;
  param.maxIterations = 500;    /* requires a larger number of iterations to converge */
  param.verbosity = NonlinearOptimizerParams::SILENT;

  NonlinearConjugateGradientOptimizer optimizer(graph, initialEstimate, param);
  Values result = optimizer.optimize();
//  cout << "cg final error = " << graph.error(result) << endl;

  /* the optimality of the solution is not comparable to the */
  DOUBLES_EQUAL(0.0, graph.error(result), 1e-2);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
