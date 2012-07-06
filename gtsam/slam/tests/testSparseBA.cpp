/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testsparseBA.cpp
 * @brief  
 * @date   Jul 5, 2012
 * @author Yong-Dian Jian
 */

#include <gtsam/slam/sparseBA.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace boost;
using namespace gtsam;

/* ************************************************************************* */

static SharedNoiseModel sigma(noiseModel::Unit::Create(2));

// Convenience for named keys
using symbol_shorthand::X;    /* pose3 */
using symbol_shorthand::K;    /* calibration */
using symbol_shorthand::C;    /* camera = [pose calibration] */
using symbol_shorthand::L;    /* point3 */

static Point3 landmark1(-1.0,-1.0, 0.0);
static Point3 landmark2(-1.0, 1.0, 0.0);
static Point3 landmark3( 1.0, 1.0, 0.0);
static Point3 landmark4( 1.0,-1.0, 0.0);

static Pose3 pose1(Matrix_(3,3,
                           1., 0., 0.,
                           0.,-1., 0.,
                           0., 0.,-1.),
                   Point3(0,0,6.25));

static Pose3 pose2(Matrix_(3,3,
                           1., 0., 0.,
                           0.,-1., 0.,
                           0., 0.,-1.),
                   Point3(0,0,5.00));

static Cal3_S2 calib1 (625, 625, 0, 0, 0);
static Cal3_S2 calib2 (625, 625, 0, 0, 0);


typedef PinholeCamera<Cal3_S2> Camera;

static Camera camera1(pose1, calib1);
static Camera camera2(pose2, calib2);

/* ************************************************************************* */
sparseBA::Graph testGraph1() {
  Point2 z11(-100, 100);
  Point2 z12(-100,-100);
  Point2 z13( 100,-100);
  Point2 z14( 100, 100);
  Point2 z21(-125, 125);
  Point2 z22(-125,-125);
  Point2 z23( 125,-125);
  Point2 z24( 125, 125);


  sparseBA::Graph g;
  g.addMeasurement<Camera>(z11, sigma, C(1), L(1));
  g.addMeasurement<Camera>(z12, sigma, C(1), L(2));
  g.addMeasurement<Camera>(z13, sigma, C(1), L(3));
  g.addMeasurement<Camera>(z14, sigma, C(1), L(4));
  g.addMeasurement<Camera>(z21, sigma, C(2), L(1));
  g.addMeasurement<Camera>(z22, sigma, C(2), L(2));
  g.addMeasurement<Camera>(z23, sigma, C(2), L(3));
  g.addMeasurement<Camera>(z24, sigma, C(2), L(4));
  return g;
}

sparseBA::Graph testGraph2() {
  Point2 z11(-100, 100);
  Point2 z12(-100,-100);
  Point2 z13( 100,-100);
  Point2 z14( 100, 100);
  Point2 z21(-125, 125);
  Point2 z22(-125,-125);
  Point2 z23( 125,-125);
  Point2 z24( 125, 125);

  sparseBA::Graph g;
  g.addMeasurement<Cal3_S2>(z11, sigma, X(1), L(1), K(1));
  g.addMeasurement<Cal3_S2>(z12, sigma, X(1), L(2), K(1));
  g.addMeasurement<Cal3_S2>(z13, sigma, X(1), L(3), K(1));
  g.addMeasurement<Cal3_S2>(z14, sigma, X(1), L(4), K(1));
  g.addMeasurement<Cal3_S2>(z21, sigma, X(2), L(1), K(1));
  g.addMeasurement<Cal3_S2>(z22, sigma, X(2), L(2), K(1));
  g.addMeasurement<Cal3_S2>(z23, sigma, X(2), L(3), K(1));
  g.addMeasurement<Cal3_S2>(z24, sigma, X(2), L(4), K(1));
  return g;
}

/* ************************************************************************* */
TEST( optimizeLM1, sparseBA )
{
  // build a graph
  sparseBA::Graph graph(testGraph1());

  // add 3 landmark constraints
  graph.addPointConstraint(L(1), landmark1);
  graph.addPointConstraint(L(2), landmark2);
  graph.addPointConstraint(L(3), landmark3);

  // Create an initial values structure corresponding to the ground truth
  Values initialEstimate;
  initialEstimate.insert(C(1), camera1);
  initialEstimate.insert(C(2), camera2);
  initialEstimate.insert(L(1), landmark1);
  initialEstimate.insert(L(2), landmark2);
  initialEstimate.insert(L(3), landmark3);
  initialEstimate.insert(L(4), landmark4);

  // Create an ordering of the variables
  Ordering ordering;
  ordering += L(1),L(2),L(3),L(4),C(1),C(2);

  // Create an optimizer and check its error
  // We expect the initial to be zero because config is the ground truth
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, ordering);
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed because we started
  // with the ground truth
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // check if correct
  CHECK(assert_equal(initialEstimate, optimizer.values()));
}

/* ************************************************************************* */
TEST( optimizeLM2, sparseBA )
{
  // build a graph
  sparseBA::Graph graph(testGraph2());

  // add 3 landmark constraints
  graph.addPointConstraint(L(1), landmark1);
  graph.addPointConstraint(L(2), landmark2);
  graph.addPointConstraint(L(3), landmark3);

  // Create an initial values structure corresponding to the ground truth
  Values initialEstimate;
  initialEstimate.insert(X(1), pose1);
  initialEstimate.insert(X(2), pose2);
  initialEstimate.insert(L(1), landmark1);
  initialEstimate.insert(L(2), landmark2);
  initialEstimate.insert(L(3), landmark3);
  initialEstimate.insert(L(4), landmark4);
  initialEstimate.insert(K(1), calib2);

  // Create an ordering of the variables
  Ordering ordering;
  ordering += L(1),L(2),L(3),L(4),X(1),X(2),K(1);

  // Create an optimizer and check its error
  // We expect the initial to be zero because config is the ground truth
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, ordering);
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed because we started
  // with the ground truth
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // check if correct
  CHECK(assert_equal(initialEstimate, optimizer.values()));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
