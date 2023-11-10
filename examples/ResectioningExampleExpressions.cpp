/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ResectioningExampleExpressions.cpp
 * @brief   Camera resectioning using expressions from a mix of 2D and 3D
 * correspondences.
 * @author  Frank Dellaert
 * @date    October 4, 2023
 */

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>

using namespace gtsam;

// Create a camera with known calibration at identity pose:
Cal3_S2 K{500, 500, 0, 320, 240};
PinholeCamera<Cal3_S2> camera(Pose3(), K);

// Three known landmarks on the Z=2 plane, i.e., in front of the camera.
Point3 landmark1(1, 1, 2), landmark2(1, -1, 2), landmark3(-1, -1, 2);

// Measurement data structures
struct Correspondence2D {
  Point2 p;
  Matrix cov2;
  Point3 landmark;
};
struct Correspondence3D {
  Point3 P;
  Matrix cov3;
  Point3 landmark;
};
static std::vector<Correspondence2D> correspondences2D;
static std::vector<Correspondence3D> correspondences3D;

// Function to create measurement data:
static void createMeasurementData() {
  // Create two 2D correspondences for landmark1 and landmark2:
  correspondences2D.push_back({camera.project(landmark1), I_2x2, landmark1});
  correspondences2D.push_back({camera.project(landmark2), I_2x2, landmark2});

  // Create a 3D correspondence for landmark3:
  correspondences3D.push_back({landmark3, I_3x3, landmark3});
}

int main() {
  // Create example data:
  createMeasurementData();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Create an expression for the unknown pose. We do this by providing a key,
  // that will index this unknown in the factor graph. By convention, we add an
  // underscore to expressions to distinguish them from regular values.
  Key poseKey = symbol('x', 0);
  Expression<Pose3> pose_(poseKey);

  // Create a constant expression for the known calibration by simply providing
  // the value. Extra credit: if you want to also optimize for calibration, just
  // use the key constructor for the expression below (with a key different from
  // the pose, of course).
  Expression<Cal3_S2> K_(K);

  // Loop over your 2D correspondences
  for (const auto& correspondence : correspondences2D) {
    Point2 p = correspondence.p;
    Matrix cov2 = correspondence.cov2;

    // Create a constant expression for the known landmark:
    Expression<Point3> landmark_(correspondence.landmark);

    // Create an expression for the measurement prediction. We use a handy-dandy
    // function overload called project3 defined in slam/expressions.h:
    Expression<Point2> prediction2 =
        project3<Cal3_S2, Point3>(pose_, landmark_, K_);
    auto noise2D = noiseModel::Gaussian::Covariance(cov2);
    graph.emplace_shared<ExpressionFactor<Point2>>(noise2D, p, prediction2);
  }

  // Loop over your 3D correspondences
  for (const auto& correspondence : correspondences3D) {
    Point3 P = correspondence.P;
    Matrix cov3 = correspondence.cov3;

    // Create a constant expression for the known landmark:
    Expression<Point3> landmark_(correspondence.landmark);

    // Create an expression for the measurement prediction. Here we use the
    // transformTo overload also defined in slam/expressions.h:
    Expression<Point3> prediction3 = transformTo(pose_, landmark_);
    auto noise3D = noiseModel::Gaussian::Covariance(cov3);
    graph.emplace_shared<ExpressionFactor<Point3>>(noise3D, P, prediction3);
  }

  // Initialize the pose estimate a little bit off:
  Values initial;
  initial.insert(poseKey,
                 Pose3(Rot3::Ypr(0.1, 0.2, 0.3), Point3(0.1, 0.2, 0.3)));

  // Create params that add some verbosity:
  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setVerbosityLM("ERROR");

  // Optimize the graph
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  Values result = optimizer.optimize();

  // Print the results
  result.print("Final result:\n");
  return 0;
}
