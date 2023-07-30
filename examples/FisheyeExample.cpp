/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FisheyeExample.cpp
 * @brief   A visualSLAM example for the structure-from-motion problem on a
 * simulated dataset. This version uses a fisheye camera model and a GaussNewton
 * solver to solve the graph in one batch
 * @author  ghaggin
 * @Date    Apr 9,2020
 */

/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 */

// For loading the data
#include "SFMdata.h"

// Camera observations of landmarks will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// Use GaussNewtonOptimizer to solve graph
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <fstream>
#include <vector>

using namespace std;
using namespace gtsam;

using symbol_shorthand::L;  // for landmarks
using symbol_shorthand::X;  // for poses

/* ************************************************************************* */
int main(int argc, char *argv[]) {
  // Define the camera calibration parameters
  auto K = std::make_shared<Cal3Fisheye>(
      278.66, 278.48, 0.0, 319.75, 241.96, -0.013721808247486035,
      0.020727425669427896, -0.012786476702685545, 0.0025242267320687625);

  // Define the camera observation noise model, 1 pixel stddev
  auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

  // Create the set of ground-truth landmarks
  const vector<Point3> points = createPoints();

  // Create the set of ground-truth poses
  const vector<Pose3> poses = createPoses();

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // Add a prior on pose x0, 0.1 rad on roll,pitch,yaw, and 30cm std on x,y,z
  auto posePrior = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
  graph.emplace_shared<PriorFactor<Pose3>>(X(0), poses[0], posePrior);

  // Add a prior on landmark l0
  auto pointPrior = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.emplace_shared<PriorFactor<Point3>>(L(0), points[0], pointPrior);

  // Add initial guesses to all observed landmarks
  // Intentionally initialize the variables off from the ground truth
  static const Point3 kDeltaPoint(-0.25, 0.20, 0.15);
  for (size_t j = 0; j < points.size(); ++j)
    initialEstimate.insert<Point3>(L(j), points[j] + kDeltaPoint);

  // Loop over the poses, adding the observations to the graph
  for (size_t i = 0; i < poses.size(); ++i) {
    // Add factors for each landmark observation
    for (size_t j = 0; j < points.size(); ++j) {
      PinholeCamera<Cal3Fisheye> camera(poses[i], *K);
      Point2 measurement = camera.project(points[j]);
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3Fisheye>>(
          measurement, measurementNoise, X(i), L(j), K);
    }

    // Add an initial guess for the current pose
    // Intentionally initialize the variables off from the ground truth
    static const Pose3 kDeltaPose(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                  Point3(0.05, -0.10, 0.20));
    initialEstimate.insert(X(i), poses[i] * kDeltaPose);
  }

  GaussNewtonParams params;
  params.setVerbosity("TERMINATION");
  params.maxIterations = 10000;

  std::cout << "Optimizing the factor graph" << std::endl;
  GaussNewtonOptimizer optimizer(graph, initialEstimate, params);
  Values result = optimizer.optimize();
  std::cout << "Optimization complete" << std::endl;

  std::cout << "initial error=" << graph.error(initialEstimate) << std::endl;
  std::cout << "final error=" << graph.error(result) << std::endl;

  graph.saveGraph("examples/vio_batch.dot", result);

  return 0;
}
/* ************************************************************************* */
