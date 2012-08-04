/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualSLAMExample.cpp
 * @brief   A visualSLAM example for the structure-from-motion problem on a simulated dataset
 * @author  Duy-Nguyen Ta
 */

/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 */

// As this is a full 3D problem, we will use Pose3 variables to represent the camera
// positions and Point3 variables (x, y, z) to represent the landmark coordinates.
// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
// We will also need a camera object to hold calibration information and perform projections.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/nonlinear/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
// trust-region method known as Powell's Degleg
#include <gtsam/nonlinear/DoglegOptimizer.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include <vector>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;
  double radius = 30.0;
  int i = 0;
  double theta = 0.0;
  gtsam::Point3 up(0,0,1);
  gtsam::Point3 target(0,0,0);
  for(; i < 8; ++i, theta += 2*M_PI/8) {
    gtsam::Point3 position = Point3(radius*cos(theta), radius*sin(theta), 0.0);
    gtsam::SimpleCamera camera = SimpleCamera::Lookat(position, target, up);
    poses.push_back(camera.pose());
  }

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(Vector_(6, 0.3, 0.3, 0.3, 0.1, 0.1, 0.1)); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.add(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise)); // add directly to graph

  // Simulated measurements from each camera pose, adding them to the factor graph
  for (size_t i = 0; i < poses.size(); ++i) {
    for (size_t j = 0; j < points.size(); ++j) {
      SimpleCamera camera(poses[i], *K);
      Point2 measurement = camera.project(points[j]);
      graph.add(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K));
    }
  }

  // Because the structure-from-motion problem has a scale ambiguity, the problem is still under-constrained
  // Here we add a prior on the position of the first landmark. This fixes the scale by indicating the distance
  // between the first camera and the first landmark. All other landmark positions are interpreted using this scale.
  noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.add(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph
  graph.print("Factor Graph:\n");

  // Create the data structure to hold the initialEstimate estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::rodriguez(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
  for (size_t j = 0; j < points.size(); ++j)
    initialEstimate.insert(Symbol('l', j), points[j].compose(Point3(-0.25, 0.20, 0.15)));
  initialEstimate.print("Initial Estimates:\n");

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  result.print("Final results:\n");

  return 0;
}
/* ************************************************************************* */

