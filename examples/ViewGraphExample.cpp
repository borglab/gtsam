/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ViewGraphExample.cpp
 * @brief   View-graph calibration on a simulated dataset, a la Sweeney 2015
 * @author  Frank Dellaert
 * @author  October 2024
 */

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/EdgeKey.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/TransferFactor.h>

#include <vector>

#include "SFMdata.h"
#include "gtsam/inference/Key.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Define the camera calibration parameters
  Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);

  // Create the set of 8 ground-truth landmarks
  vector<Point3> points = createPoints();

  // Create the set of 4 ground-truth poses
  vector<Pose3> poses = posesOnCircle(4, 30);

  // Calculate ground truth fundamental matrices, 1 and 2 poses apart
  auto F1 = FundamentalMatrix(K, poses[0].between(poses[1]), K);
  auto F2 = FundamentalMatrix(K, poses[0].between(poses[2]), K);

  // Simulate measurements from each camera pose
  std::array<std::array<Point2, 8>, 4> p;
  for (size_t i = 0; i < 4; ++i) {
    PinholeCamera<Cal3_S2> camera(poses[i], K);
    for (size_t j = 0; j < 8; ++j) {
      p[i][j] = camera.project(points[j]);
    }
  }

  // This section of the code is inspired by the work of Sweeney et al.
  // [link](sites.cs.ucsb.edu/~holl/pubs/Sweeney-2015-ICCV.pdf) on view-graph
  // calibration. The graph is made up of transfer factors that enforce the
  // epipolar constraint between corresponding points across three views, as
  // described in the paper. Rather than adding one ternary error term per point
  // in a triplet, we add three binary factors for sparsity during optimization.
  // In this version, we only include triplets between 3 successive cameras.
  NonlinearFactorGraph graph;
  using Factor = TransferFactor<FundamentalMatrix>;
  for (size_t a = 0; a < 4; ++a) {
    size_t b = (a + 1) % 4;  // Next camera
    size_t c = (a + 2) % 4;  // Camera after next
    for (size_t j = 0; j < 4; ++j) {
      // Add transfer factors between views a, b, and c. Note that the EdgeKeys
      // are crucial in performing the transfer in the right direction. We use
      // exactly 8 unique EdgeKeys, corresponding to 8 unknown fundamental
      // matrices we will optimize for.
      graph.emplace_shared<Factor>(EdgeKey(a, c), EdgeKey(b, c), p[a][j],
                                   p[b][j], p[c][j]);
      graph.emplace_shared<Factor>(EdgeKey(a, b), EdgeKey(b, c), p[a][j],
                                   p[c][j], p[b][j]);
      graph.emplace_shared<Factor>(EdgeKey(a, c), EdgeKey(a, b), p[c][j],
                                   p[b][j], p[a][j]);
    }
  }

  auto formatter = [](Key key) {
    EdgeKey edge(key);
    return (std::string)edge;
  };

  graph.print("Factor Graph:\n", formatter);

  // Create a delta vector to perturb the ground truth
  // We can't really go far before convergence becomes problematic :-(
  Vector7 delta;
  delta << 1, 2, 3, 4, 5, 6, 7;
  delta *= 5e-5;

  // Create the data structure to hold the initial estimate to the solution
  Values initialEstimate;
  for (size_t a = 0; a < 4; ++a) {
    size_t b = (a + 1) % 4;  // Next camera
    size_t c = (a + 2) % 4;  // Camera after next
    initialEstimate.insert(EdgeKey(a, b), F1.retract(delta));
    initialEstimate.insert(EdgeKey(a, c), F2.retract(delta));
  }
  initialEstimate.print("Initial Estimates:\n", formatter);
  //   graph.printErrors(initialEstimate, "errors: ", formatter);

  /* Optimize the graph and print results */
  LevenbergMarquardtParams params;
  params.setlambdaInitial(1000.0);  // Initialize lambda to a high value
  params.setVerbosityLM("SUMMARY");
  Values result =
      LevenbergMarquardtOptimizer(graph, initialEstimate, params).optimize();

  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;

  result.print("Final results:\n", formatter);

  cout << "Ground Truth F1:\n" << F1.matrix() << endl;
  cout << "Ground Truth F2:\n" << F2.matrix() << endl;

  return 0;
}
/* ************************************************************************* */
