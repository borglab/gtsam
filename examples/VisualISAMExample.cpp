/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualISAMExample.cpp
 * @brief   A visualSLAM example for the structure-from-motion problem on a simulated dataset
 * This version uses iSAM to solve the problem incrementally
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

// We want to use iSAM to solve the structure-from-motion problem incrementally, so
// include iSAM here
#include <gtsam/nonlinear/NonlinearISAM.h>

// iSAM requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
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

  // Create a NonlinearISAM object which will relinearize and reorder the variables every "relinearizeInterval" updates
  int relinearizeInterval = 3;
  NonlinearISAM isam(relinearizeInterval);

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // Loop over the different poses, adding the observations to iSAM incrementally
  for (size_t i = 0; i < poses.size(); ++i) {

    // Add factors for each landmark observation
    for (size_t j = 0; j < points.size(); ++j) {
      SimpleCamera camera(poses[i], *K);
      Point2 measurement = camera.project(points[j]);
      graph.add(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K));
    }

    // Add an initial guess for the current pose
    // Intentionally initialize the variables off from the ground truth
    initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::rodriguez(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));

    // If this is the first iteration, add a prior on the first pose to set the coordinate frame
    // and a prior on the first landmark to set the scale
    // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
    // adding it to iSAM.
    if( i == 0) {
      // Add a prior on pose x0
      noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(Vector_(6, 0.3, 0.3, 0.3, 0.1, 0.1, 0.1)); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
      graph.add(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));

      // Add a prior on landmark l0
      noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
      graph.add(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph

      // Add initial guesses to all observed landmarks
      // Intentionally initialize the variables off from the ground truth
      for (size_t j = 0; j < points.size(); ++j)
        initialEstimate.insert(Symbol('l', j), points[j].compose(Point3(-0.25, 0.20, 0.15)));

    } else {
      // Update iSAM with the new factors
      isam.update(graph, initialEstimate);
      Values currentEstimate = isam.estimate();
      cout << "****************************************************" << endl;
      cout << "Frame " << i << ": " << endl;
      currentEstimate.print("Current estimate: ");

      // Clear the factor graph and values for the next iteration
      graph.resize(0);
      initialEstimate.clear();
    }
  }

  return 0;
}
/* ************************************************************************* */
