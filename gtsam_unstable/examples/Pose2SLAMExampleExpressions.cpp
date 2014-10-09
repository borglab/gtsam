/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample.cpp
 * @brief Expressions version of Pose2SLAMExample.cpp
 * @date Oct 2, 2014
 * @author Frank Dellaert
 * @author Yong Dian Jian
 */

// The two new headers that allow using our Automatic Differentiation Expression framework
#include <gtsam_unstable/slam/expressions.h>
#include <gtsam_unstable/nonlinear/ExpressionFactor.h>

// Header order is close to far
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // Create Expressions for unknowns
  Pose2_ x1(1), x2(2), x3(3), x4(4), x5(5);

  // 2a. Add a prior on the first pose, setting it to the origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
  graph.push_back(ExpressionFactor<Pose2>(priorNoise, Pose2(0, 0, 0), x1));

  // For simplicity, we will use the same noise model for odometry and loop closures
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));

  // 2b. Add odometry factors
  graph.push_back(ExpressionFactor<Pose2>(model, Pose2(2, 0, 0     ), between(x1,x2)));
  graph.push_back(ExpressionFactor<Pose2>(model, Pose2(2, 0, M_PI_2), between(x2,x3)));
  graph.push_back(ExpressionFactor<Pose2>(model, Pose2(2, 0, M_PI_2), between(x3,x4)));
  graph.push_back(ExpressionFactor<Pose2>(model, Pose2(2, 0, M_PI_2), between(x4,x5)));

  // 2c. Add the loop closure constraint
  graph.push_back(ExpressionFactor<Pose2>(model, Pose2(2, 0, M_PI_2), between(x5,x2)));
  graph.print("\nFactor Graph:\n"); // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0,  0.2   ));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2   ));
  initialEstimate.insert(3, Pose2(4.1, 0.1,  M_PI_2));
  initialEstimate.insert(4, Pose2(4.0, 2.0,  M_PI  ));
  initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
  GaussNewtonParams parameters;
  parameters.relativeErrorTol = 1e-5;
  parameters.maxIterations = 100;
  GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  cout.precision(3);
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
  cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;

  return 0;
}
