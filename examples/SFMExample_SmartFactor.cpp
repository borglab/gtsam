/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample_SmartFactor.cpp
 * @brief   A structure-from-motion problem on a simulated dataset, using smart projection factor
 * @author  Duy-Nguyen Ta
 * @author  Jing Dong
 */

/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 */

// For loading the data
#include "SFMdata.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'.
// The factor we used here is SmartProjectionPoseFactor. Every smart factor represent a single landmark,
// The SmartProjectionPoseFactor only optimize the pose of camera, not the calibration,
// The calibration should be known.
#include <gtsam/slam/SmartProjectionPoseFactor.h>

// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>

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

// Make the typename short so it looks much cleaner
typedef gtsam::SmartProjectionPoseFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
  SmartFactor;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();

  // Create the set of ground-truth poses
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // A vector saved all Smart factors (for get landmark position after optimization)
  vector<SmartFactor::shared_ptr> smartfactors_ptr;

  // Simulated measurements from each camera pose, adding them to the factor graph
  for (size_t i = 0; i < points.size(); ++i) {

    // every landmark represent a single landmark, we use shared pointer to init the factor, and then insert measurements.
    SmartFactor::shared_ptr smartfactor(new SmartFactor());

    for (size_t j = 0; j < poses.size(); ++j) {

      // generate the 2D measurement
      SimpleCamera camera(poses[j], *K);
      Point2 measurement = camera.project(points[i]);

      // call add() function to add measurment into a single factor, here we need to add:
      //    1. the 2D measurement
      //    2. the corresponding camera's key
      //    3. camera noise model
      //    4. camera calibration
      smartfactor->add(measurement, Symbol('x', j), measurementNoise, K);
    }

    // save smartfactors to get landmark position
    smartfactors_ptr.push_back(smartfactor);

    // insert the smart factor in the graph
    graph.push_back(smartfactor);
  }

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1))); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise)); // add directly to graph

  // Because the structure-from-motion problem has a scale ambiguity, the problem is still under-constrained
  // Here we add a prior on the second pose x1, so this will fix the scale by indicating the distance between x0 and x1.
  // Because these two are fixed, the rest poses will be alse fixed.
  graph.push_back(PriorFactor<Pose3>(Symbol('x', 1), poses[1], poseNoise)); // add directly to graph

  graph.print("Factor Graph:\n");

  // Create the data structure to hold the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::rodriguez(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
  initialEstimate.print("Initial Estimates:\n");

  // Optimize the graph and print results
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  result.print("Final results:\n");


  // Notice: Smart factor represent the 3D point as a factor, not a variable.
  // The 3D position of the landmark is not explicitly calculated by the optimizer.
  // If you do want to output the landmark's 3D position, you should use the internal function point()
  // of the smart factor to get the 3D point.
  Values landmark_result;
  for (size_t i = 0; i < points.size(); ++i) {

    // The output of point() is in boost::optional<gtsam::Point3>, since sometimes
    // the triangulation opterations inside smart factor will encounter degeneracy.
    // Check the manual of boost::optional for more details for the usages.
    boost::optional<Point3> point;

    // here we use the saved smart factors to call, pass in all optimized pose to calculate landmark positions
    point = smartfactors_ptr.at(i)->point(result);

    // ignore if boost::optional return NULL
    if (point)
      landmark_result.insert(Symbol('l', i), *point);
  }

  landmark_result.print("Landmark results:\n");


  return 0;
}
/* ************************************************************************* */

