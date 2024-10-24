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
#include "gtsam/geometry/EssentialMatrix.h"
#include "gtsam/inference/Key.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Create the set of 8 ground-truth landmarks
  vector<Point3> points = createPoints();

  // Create the set of 4 ground-truth poses
  vector<Pose3> poses = posesOnCircle(4, 30);

  // Simulate measurements from each camera pose
  std::array<std::array<Point2, 8>, 4> p;
  for (size_t i = 0; i < 4; ++i) {
    GTSAM_PRINT(poses[i]);
    PinholeCamera<Cal3_S2> camera(poses[i], *K);
    for (size_t j = 0; j < 8; ++j) {
      cout << "Camera index: " << i << ", Landmark index: " << j << endl;
      p[i][j] = camera.project(points[j]);
    }
  }

  // Create a factor graph
  NonlinearFactorGraph graph;

  using Factor = TransferFactor<SimpleFundamentalMatrix>;
  for (size_t a = 0; a < 4; ++a) {
    size_t b = (a + 1) % 4;  // Next camera
    size_t c = (a + 2) % 4;  // Camera after next
    for (size_t j = 0; j < 4; ++j) {
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

  // Create the data structure to hold the initial estimate to the solution
  Values initialEstimate;
  const Point2 center(50, 50);
  auto E1 = EssentialMatrix::FromPose3(poses[0].between(poses[1]));
  auto E2 = EssentialMatrix::FromPose3(poses[0].between(poses[2]));
  for (size_t a = 0; a < 4; ++a) {
    size_t b = (a + 1) % 4;  // Next camera
    size_t c = (a + 2) % 4;  // Camera after next
    initialEstimate.insert(EdgeKey(a, b),
                           SimpleFundamentalMatrix(E1, 50, 50, center, center));
    initialEstimate.insert(EdgeKey(a, c),
                           SimpleFundamentalMatrix(E2, 50, 50, center, center));
  }
  initialEstimate.print("Initial Estimates:\n", formatter);
  //   graph.printErrors(initialEstimate, "errors: ", formatter);

  /* Optimize the graph and print results */
  LevenbergMarquardtParams params;
  params.setlambdaInitial(1000.0);  // Initialize lambda to a high value
  params.setVerbosityLM("SUMMARY");
  Values result =
      LevenbergMarquardtOptimizer(graph, initialEstimate, params).optimize();
  result.print("Final results:\n", formatter);
  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;

  return 0;
}
/* ************************************************************************* */
