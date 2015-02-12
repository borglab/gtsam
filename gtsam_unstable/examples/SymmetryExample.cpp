/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymmetryExample.cpp
 * @brief   Optimize for a 3D point in a symmetric structure
 * @author  Frank Dellaert
 * @author  Natesh Srinivasan
 * @date    Feb 12, 2015
 */

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

//#include <vector>

using namespace std;
using namespace gtsam;
using namespace noiseModel;

/* ************************************************************************* */
// A local function that just adds two points
static Point3 add(const Point3& p, const Point3& q, //
    OptionalJacobian<3, 3> H1, OptionalJacobian<3, 3> H2) {
  if (H1)
    *H1 = I_3x3;
  if (H2)
    *H2 = I_3x3;
  return p + q;
}
/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // Create calibration and noise model
  Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);
  Isotropic::shared_ptr measurementNoise = Isotropic::Sigma(2, 1.0);

  // =========== Create a ground truth example with symmetry ===================

  // We create one 3D point that is translated by t to generate a second point
  // So we have two "real" points
  vector<Point3> points;
  Point3 p(0, 0, 10), t(5, 0, 0);
  points.push_back(p);
  points.push_back(p + t);

  // Just use a known camera at the origin
  SimpleCamera camera(Pose3(), K);

  // Simulated measurements
  vector<Point2> measurements;
  for (size_t j = 0; j < points.size(); ++j) {
    Point2 measurement = camera.project(points[j]);
    measurements.push_back(measurement);
  }

  // ======================== Do inference =====================================

  // Create a factor graph
  ExpressionFactorGraph graph;

  // We create a constant Expression for the camera here
  Expression<SimpleCamera> constantCameraExpression(camera);

  // and also for the known translation
  Point3_ constantTranslationExpression(t);

  // Create an expression for the unknown point by calling the constructor
  // with a key, in this case, 42. Note, there is only *one* unknown point,
  // because we know that the measurements originated from symmetric points.
  // For this simple example, we use the first occurrence of the repeated
  // point as the unknown to optimize for.
  Point3_ unknownPointExpression(42);

  // Add a factor for each measurement
  for (size_t j = 0; j < measurements.size(); ++j) {
    Point2 measurement = measurements[j];
    // Create an expression for the prediction of the measurement,
    // and add to ExpressionFactorGraph with handy-dandy method...
    if (j == 0) {
      Point2_ prediction = //
          project2(constantCameraExpression, unknownPointExpression);
      graph.addExpressionFactor(prediction, measurement, measurementNoise);
    } else {
      Point3_ displacedPointExpression(add, unknownPointExpression,
          constantTranslationExpression);
      Point2_ prediction = //
          project2(constantCameraExpression, displacedPointExpression);
      graph.addExpressionFactor(prediction, measurement, measurementNoise);
    }
  }

  // Create perturbed initial
  Values initial;
  initial.insert(42, Point3(1, 2, 3));
  cout << "initial error = " << graph.error(initial) << endl;

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initial).optimize();
  cout << "final error = " << graph.error(result) << endl;
  result.print("result");

  return 0;
}
/* ************************************************************************* */

