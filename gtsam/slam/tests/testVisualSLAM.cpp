/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testVisualSLAM.cpp
 * @brief   Unit test for two cameras and four landmarks, single camera
 * @author  Chris Beall
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/shared_ptr.hpp>
using namespace boost;

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

static SharedNoiseModel sigma(noiseModel::Unit::Create(1));

// Convenience for named keys
Key kx(size_t i) { return Symbol('x',i); }
Key kl(size_t i) { return Symbol('l',i); }

static Point3 landmark1(-1.0,-1.0, 0.0);
static Point3 landmark2(-1.0, 1.0, 0.0);
static Point3 landmark3( 1.0, 1.0, 0.0);
static Point3 landmark4( 1.0,-1.0, 0.0);

static Pose3 camera1(Matrix_(3,3,
		       1., 0., 0.,
		       0.,-1., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,6.25));

static Pose3 camera2(Matrix_(3,3,
		       1., 0., 0.,
		       0.,-1., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,5.00));

/* ************************************************************************* */
visualSLAM::Graph testGraph() {
  Point2 z11(-100, 100);
	Point2 z12(-100,-100);
	Point2 z13( 100,-100);
	Point2 z14( 100, 100);
	Point2 z21(-125, 125);
	Point2 z22(-125,-125);
	Point2 z23( 125,-125);
	Point2 z24( 125, 125);

  shared_ptrK sK(new Cal3_S2(625, 625, 0, 0, 0));
  visualSLAM::Graph g;
  g.addMeasurement(z11, sigma, kx(1), kl(1), sK);
  g.addMeasurement(z12, sigma, kx(1), kl(2), sK);
  g.addMeasurement(z13, sigma, kx(1), kl(3), sK);
  g.addMeasurement(z14, sigma, kx(1), kl(4), sK);
  g.addMeasurement(z21, sigma, kx(2), kl(1), sK);
  g.addMeasurement(z22, sigma, kx(2), kl(2), sK);
  g.addMeasurement(z23, sigma, kx(2), kl(3), sK);
  g.addMeasurement(z24, sigma, kx(2), kl(4), sK);
  return g;
}

/* ************************************************************************* */
TEST( Graph, optimizeLM)
{
  // build a graph
  visualSLAM::Graph graph(testGraph());
	// add 3 landmark constraints
  graph.addPointConstraint(kl(1), landmark1);
  graph.addPointConstraint(kl(2), landmark2);
  graph.addPointConstraint(kl(3), landmark3);

  // Create an initial values structure corresponding to the ground truth
  Values initialEstimate;
  initialEstimate.insert(kx(1), camera1);
  initialEstimate.insert(kx(2), camera2);
  initialEstimate.insert(kl(1), landmark1);
  initialEstimate.insert(kl(2), landmark2);
  initialEstimate.insert(kl(3), landmark3);
  initialEstimate.insert(kl(4), landmark4);

  // Create an ordering of the variables
  Ordering ordering;
  ordering += kl(1),kl(2),kl(3),kl(4),kx(1),kx(2);

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
TEST( Graph, optimizeLM2)
{
  // build a graph
  visualSLAM::Graph graph(testGraph());
	// add 2 camera constraints
  graph.addPoseConstraint(kx(1), camera1);
  graph.addPoseConstraint(kx(2), camera2);

  // Create an initial values structure corresponding to the ground truth
  Values initialEstimate;
  initialEstimate.insert(kx(1), camera1);
  initialEstimate.insert(kx(2), camera2);
  initialEstimate.insert(kl(1), landmark1);
  initialEstimate.insert(kl(2), landmark2);
  initialEstimate.insert(kl(3), landmark3);
  initialEstimate.insert(kl(4), landmark4);

  // Create an ordering of the variables
  Ordering ordering;
  ordering += kl(1),kl(2),kl(3),kl(4),kx(1),kx(2);

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
TEST( Graph, CHECK_ORDERING)
{
  // build a graph
  visualSLAM::Graph graph = testGraph();
  // add 2 camera constraints
  graph.addPoseConstraint(kx(1), camera1);
  graph.addPoseConstraint(kx(2), camera2);

  // Create an initial values structure corresponding to the ground truth
  Values initialEstimate;
  initialEstimate.insert(kx(1), camera1);
  initialEstimate.insert(kx(2), camera2);
  initialEstimate.insert(kl(1), landmark1);
  initialEstimate.insert(kl(2), landmark2);
  initialEstimate.insert(kl(3), landmark3);
  initialEstimate.insert(kl(4), landmark4);

  // Create an optimizer and check its error
  // We expect the initial to be zero because config is the ground truth
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed because we started
  // with the ground truth
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // check if correct
  CHECK(assert_equal(initialEstimate, optimizer.values()));
}

/* ************************************************************************* */
TEST( Values, update_with_large_delta) {
	// this test ensures that if the update for delta is larger than
	// the size of the config, it only updates existing variables
	Values init;
	init.insert(kx(1), Pose3());
	init.insert(kl(1), Point3(1.0, 2.0, 3.0));

	Values expected;
	expected.insert(kx(1), Pose3(Rot3(), Point3(0.1, 0.1, 0.1)));
	expected.insert(kl(1), Point3(1.1, 2.1, 3.1));

	Ordering largeOrdering;
	Values largeValues = init;
	largeValues.insert(kx(2), Pose3());
	largeOrdering += kx(1),kl(1),kx(2);
	VectorValues delta(largeValues.dims(largeOrdering));
	delta[largeOrdering[kx(1)]] = Vector_(6, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1);
	delta[largeOrdering[kl(1)]] = Vector_(3, 0.1, 0.1, 0.1);
	delta[largeOrdering[kx(2)]] = Vector_(6, 0.0, 0.0, 0.0, 100.1, 4.1, 9.1);
	Values actual = init.retract(delta, largeOrdering);

	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
