/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMwSPCG.cpp
 * @brief A 2D Pose SLAM example using the SimpleSPCGSolver.
 * @author Yong-Dian Jian
 * @date June 2, 2012
 */

/**
 * A simple 2D pose slam example solved using a Conjugate-Gradient method
 *  - The robot moves in a 2 meter square
 *  - The robot moves 2 meters each step, turning 90 degrees after each step
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have a loop closure constraint when the robot returns to the first position
 */

// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use simple integer keys
#include <gtsam/nonlinear/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  Pose2 prior(0.0, 0.0, 0.0); // prior at origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(1, prior, priorNoise));

  // 2b. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
  // Create odometry (Between) factors between consecutive poses
  graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2.0, 0.0, M_PI_2),    odometryNoise));
  graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2.0, 0.0, M_PI_2), odometryNoise));
  graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2.0, 0.0, M_PI_2), odometryNoise));
  graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2.0, 0.0, M_PI_2), odometryNoise));

  // 2c. Add the loop closure constraint
  // This factor encodes the fact that we have returned to the same pose. In real systems,
  // these constraints may be identified in many ways, such as appearance-based techniques
  // with camera images.
  // We will use another Between Factor to enforce this constraint, with the distance set to zero,
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
  graph.add(BetweenFactor<Pose2>(5, 1, Pose2(0.0, 0.0, 0.0), model));
  graph.print("\nFactor Graph:\n"); // print


  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, 1.1));
  initialEstimate.insert(3, Pose2(2.1, 1.9, 2.8));
  initialEstimate.insert(4, Pose2(-.3, 2.5, 4.2));
  initialEstimate.insert(5, Pose2(0.1,-0.7, 5.8));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. Single Step Optimization using Levenberg-Marquardt
  LevenbergMarquardtParams parameters;
  parameters.verbosity = NonlinearOptimizerParams::ERROR;
  parameters.verbosityLM = LevenbergMarquardtParams::LAMBDA;
  parameters.linearSolverType = SuccessiveLinearizationParams::CONJUGATE_GRADIENT;

  {
    parameters.iterativeParams = boost::make_shared<SubgraphSolverParameters>();
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");
    cout << "subgraph solver final error = " << graph.error(result) << endl;
  }

  return 0;
}
