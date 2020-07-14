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
 * @author  Frank Dellaert
 */

// In GTSAM, measurement functions are represented as 'factors'.
// The factor we used here is SmartProjectionPoseFactor.
// Every smart factor represent a single landmark, seen from multiple cameras.
// The SmartProjectionPoseFactor only optimizes for the poses of a camera,
// not the calibration, which is assumed known.
#include <gtsam/slam/SmartProjectionPoseFactor.h>

// For an explanation of these headers, see SFMExample.cpp
#include "SFMdata.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

// Make the typename short so it looks much cleaner
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

// create a typedef to the camera type
typedef PinholePose<Cal3_S2> Camera;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  auto measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Create the set of ground-truth landmarks and poses
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Simulated measurements from each camera pose, adding them to the factor graph
  for (size_t j = 0; j < points.size(); ++j) {

    // every landmark represent a single landmark, we use shared pointer to init the factor, and then insert measurements.
    SmartFactor::shared_ptr smartfactor(new SmartFactor(measurementNoise, K));

    for (size_t i = 0; i < poses.size(); ++i) {

      // generate the 2D measurement
      Camera camera(poses[i], K);
      Point2 measurement = camera.project(points[j]);

      // call add() function to add measurement into a single factor, here we need to add:
      //    1. the 2D measurement
      //    2. the corresponding camera's key
      //    3. camera noise model
      //    4. camera calibration
      smartfactor->add(measurement, i);
    }

    // insert the smart factor in the graph
    graph.push_back(smartfactor);
  }

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  auto noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
  graph.addPrior(0, poses[0], noise);

  // Because the structure-from-motion problem has a scale ambiguity, the problem is
  // still under-constrained. Here we add a prior on the second pose x1, so this will
  // fix the scale by indicating the distance between x0 and x1.
  // Because these two are fixed, the rest of the poses will be also be fixed.
  graph.addPrior(1, poses[1], noise); // add directly to graph

  graph.print("Factor Graph:\n");

  // Create the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate;
  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(i, poses[i].compose(delta));
  initialEstimate.print("Initial Estimates:\n");

  // Optimize the graph and print results
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final results:\n");

  // A smart factor represent the 3D point inside the factor, not as a variable.
  // The 3D position of the landmark is not explicitly calculated by the optimizer.
  // To obtain the landmark's 3D position, we use the "point" method of the smart factor.
  Values landmark_result;
  for (size_t j = 0; j < points.size(); ++j) {

    // The graph stores Factor shared_ptrs, so we cast back to a SmartFactor first
    SmartFactor::shared_ptr smart = boost::dynamic_pointer_cast<SmartFactor>(graph[j]);
    if (smart) {
      // The output of point() is in boost::optional<Point3>, as sometimes
      // the triangulation operation inside smart factor will encounter degeneracy.
      boost::optional<Point3> point = smart->point(result);
      if (point) // ignore if boost::optional return nullptr
        landmark_result.insert(j, *point);
    }
  }

  landmark_result.print("Landmark results:\n");
  cout << "final error: " << graph.error(result) << endl;
  cout << "number of iterations: " << optimizer.iterations() << endl;

  return 0;
}
/* ************************************************************************* */

