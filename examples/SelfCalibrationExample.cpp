/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SelfCalibrationExample.cpp
 * @brief   Based on VisualSLAMExample, but with unknown (yet fixed) calibration.
 * @author  Frank Dellaert
 */

/*
 * See the detailed documentation in Visual SLAM.
 * The only documentation below with deal with the self-calibration.
 */

// For loading the data
#include "SFMdata.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Inference and optimization
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

// SFM-specific factors
#include <gtsam/slam/GeneralSFMFactor.h>  // does calibration !

// Standard headers
#include <vector>

using namespace std;
using namespace gtsam;

int main(int argc, char* argv[]) {
  // Create the set of ground-truth
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  // Create the factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1.
  auto poseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
          .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.addPrior(Symbol('x', 0), poses[0], poseNoise);

  // Simulated measurements from each camera pose, adding them to the factor
  // graph
  Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);
  auto measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0);
  for (size_t i = 0; i < poses.size(); ++i) {
    for (size_t j = 0; j < points.size(); ++j) {
      PinholeCamera<Cal3_S2> camera(poses[i], K);
      Point2 measurement = camera.project(points[j]);
      // The only real difference with the Visual SLAM example is that here we
      // use a different factor type, that also calculates the Jacobian with
      // respect to calibration
      graph.emplace_shared<GeneralSFMFactor2<Cal3_S2> >(
          measurement, measurementNoise, Symbol('x', i), Symbol('l', j),
          Symbol('K', 0));
    }
  }

  // Add a prior on the position of the first landmark.
  auto pointNoise =
      noiseModel::Isotropic::Sigma(3, 0.1);
  graph.addPrior(Symbol('l', 0), points[0],
                 pointNoise);  // add directly to graph

  // Add a prior on the calibration.
  auto calNoise = noiseModel::Diagonal::Sigmas(
      (Vector(5) << 500, 500, 0.1, 100, 100).finished());
  graph.addPrior(Symbol('K', 0), K, calNoise);

  // Create the initial estimate to the solution
  // now including an estimate on the camera calibration parameters
  Values initialEstimate;
  initialEstimate.insert(Symbol('K', 0), Cal3_S2(60.0, 60.0, 0.0, 45.0, 45.0));
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(
        Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                               Point3(0.05, -0.10, 0.20))));
  for (size_t j = 0; j < points.size(); ++j)
    initialEstimate.insert<Point3>(Symbol('l', j),
                                   points[j] + Point3(-0.25, 0.20, 0.15));

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  result.print("Final results:\n");

  return 0;
}

