/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExampleExpressions.cpp
 * @brief   A structure-from-motion example done with Expressions
 * @author  Frank Dellaert
 * @author  Duy-Nguyen Ta
 * @date    October 1, 2014
 */

/**
 * This is the Expression version of SFMExample
 * See detailed description of headers there, this focuses on explaining the AD part
 */

// The two new headers that allow using our Automatic Differentiation Expression framework
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Header order is close to far
#include "SFMdata.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <vector>

using namespace std;
using namespace gtsam;
using namespace noiseModel;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);
  Isotropic::shared_ptr measurementNoise = Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Create the set of ground-truth landmarks and poses
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  ExpressionFactorGraph graph;

  // Specify uncertainty on first pose prior
  Vector6 sigmas; sigmas << Vector3(0.3,0.3,0.3), Vector3(0.1,0.1,0.1);
  Diagonal::shared_ptr poseNoise = Diagonal::Sigmas(sigmas);

  // Here we don't use a PriorFactor but directly the ExpressionFactor class
  // x0 is an Expression, and we create a factor wanting it to be equal to poses[0]
  Pose3_ x0('x',0);
  graph.addExpressionFactor(x0, poses[0], poseNoise);

  // We create a constant Expression for the calibration here
  Cal3_S2_ cK(K);

  // Simulated measurements from each camera pose, adding them to the factor graph
  for (size_t i = 0; i < poses.size(); ++i) {
    Pose3_ x('x', i);
    PinholeCamera<Cal3_S2> camera(poses[i], K);
    for (size_t j = 0; j < points.size(); ++j) {
      Point2 measurement = camera.project(points[j]);
      // Below an expression for the prediction of the measurement:
      Point3_ p('l', j);
      Point2_ prediction = uncalibrate(cK, project(transformTo(x, p)));
      // Again, here we use an ExpressionFactor
      graph.addExpressionFactor(prediction, measurement, measurementNoise);
    }
  }

  // Add prior on first point to constrain scale, again with ExpressionFactor
  Isotropic::shared_ptr pointNoise = Isotropic::Sigma(3, 0.1);
  graph.addExpressionFactor(Point3_('l', 0), points[0], pointNoise);

  // Create perturbed initial
  Values initial;
  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
  for (size_t i = 0; i < poses.size(); ++i)
    initial.insert(Symbol('x', i), poses[i].compose(delta));
  for (size_t j = 0; j < points.size(); ++j)
    initial.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));
  cout << "initial error = " << graph.error(initial) << endl;

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initial).optimize();
  cout << "final error = " << graph.error(result) << endl;

  return 0;
}
/* ************************************************************************* */

