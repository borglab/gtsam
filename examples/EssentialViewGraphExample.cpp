/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    EssentialViewGraphExample.cpp
 * @brief   View-graph calibration with essential matrices.
 * @author  Frank Dellaert
 * @date    October 2024
 */

#include <gtsam/geometry/Cal3f.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/EdgeKey.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/TransferFactor.h>  // Contains EssentialTransferFactorK

#include <vector>

#include "SFMdata.h"  // For createPoints() and posesOnCircle()

using namespace std;
using namespace gtsam;
using namespace symbol_shorthand;  // For K(symbol)

// Main function
int main(int argc, char* argv[]) {
  // Define the camera calibration parameters
  Cal3f cal(50.0, 50.0, 50.0);

  // Create the set of 8 ground-truth landmarks
  vector<Point3> points = createPoints();

  // Create the set of 4 ground-truth poses
  vector<Pose3> poses = posesOnCircle(4, 30);

  // Calculate ground truth essential matrices, 1 and 2 poses apart
  auto E1 = EssentialMatrix::FromPose3(poses[0].between(poses[1]));
  auto E2 = EssentialMatrix::FromPose3(poses[0].between(poses[2]));

  // Simulate measurements from each camera pose
  std::array<std::array<Point2, 8>, 4> p;
  for (size_t i = 0; i < 4; ++i) {
    PinholeCamera<Cal3f> camera(poses[i], cal);
    for (size_t j = 0; j < 8; ++j) {
      p[i][j] = camera.project(points[j]);
    }
  }

  // Create the factor graph
  NonlinearFactorGraph graph;
  using Factor = EssentialTransferFactorK<Cal3f>;

  for (size_t a = 0; a < 4; ++a) {
    size_t b = (a + 1) % 4;  // Next camera
    size_t c = (a + 2) % 4;  // Camera after next

    // Vectors to collect tuples for each factor
    std::vector<std::tuple<Point2, Point2, Point2>> tuples1, tuples2, tuples3;

    // Collect data for the three factors
    for (size_t j = 0; j < 8; ++j) {
      tuples1.emplace_back(p[a][j], p[b][j], p[c][j]);
      tuples2.emplace_back(p[a][j], p[c][j], p[b][j]);
      tuples3.emplace_back(p[c][j], p[b][j], p[a][j]);
    }

    // Add transfer factors between views a, b, and c.
    graph.emplace_shared<Factor>(EdgeKey(a, c), EdgeKey(b, c), tuples1);
    graph.emplace_shared<Factor>(EdgeKey(a, b), EdgeKey(b, c), tuples2);
    graph.emplace_shared<Factor>(EdgeKey(a, c), EdgeKey(a, b), tuples3);
  }

  // Formatter for printing keys
  auto formatter = [](Key key) {
    if (Symbol(key).chr() == 'k') {
      return (string)Symbol(key);
    } else {
      EdgeKey edge(key);
      return (std::string)edge;
    }
  };

  graph.print("Factor Graph:\n", formatter);

  // Create a delta vector to perturb the ground truth (small perturbation)
  Vector5 delta;
  delta << 1, 1, 1, 1, 1;
  delta *= 1e-2;

  // Create the initial estimate for essential matrices
  Values initialEstimate;
  for (size_t a = 0; a < 4; ++a) {
    size_t b = (a + 1) % 4;  // Next camera
    size_t c = (a + 2) % 4;  // Camera after next
    initialEstimate.insert(EdgeKey(a, b), E1.retract(delta));
    initialEstimate.insert(EdgeKey(a, c), E2.retract(delta));
  }

  // Insert initial calibrations (using K symbol)
  for (size_t i = 0; i < 4; ++i) {
    initialEstimate.insert(K(i), cal);
  }

  initialEstimate.print("Initial Estimates:\n", formatter);
  graph.printErrors(initialEstimate, "Initial Errors:\n", formatter);

  // Optimize the graph and print results
  LevenbergMarquardtParams params;
  params.setlambdaInitial(1000.0);  // Initialize lambda to a high value
  params.setVerbosityLM("SUMMARY");
  Values result =
      LevenbergMarquardtOptimizer(graph, initialEstimate, params).optimize();

  cout << "Initial error = " << graph.error(initialEstimate) << endl;
  cout << "Final error = " << graph.error(result) << endl;

  result.print("Final Results:\n", formatter);

  E1.print("Ground Truth E1:\n");
  E2.print("Ground Truth E2:\n");

  return 0;
}