/**
 * @file    testVSLAMGraph.cpp
 * @brief   Unit test for two cameras and four landmarks
 * single camera
 * @author  Chris Beall
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#include <CppUnitLite/TestHarness.h>

#include "VSLAMGraph.h"
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"

using namespace std;
using namespace gtsam;
typedef NonlinearOptimizer<VSLAMGraph,VSLAMConfig> Optimizer;

/* ************************************************************************* */
Point3 landmark1(-1.0,-1.0, 0.0);
Point3 landmark2(-1.0, 1.0, 0.0);
Point3 landmark3( 1.0, 1.0, 0.0);
Point3 landmark4( 1.0,-1.0, 0.0);

Pose3 camera1(Matrix_(3,3,
		       1., 0., 0.,
		       0.,-1., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,6.25));

Pose3 camera2(Matrix_(3,3,
		       1., 0., 0.,
		       0.,-1., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,5.00));

/* ************************************************************************* */
VSLAMGraph testGraph() {
  Point2 z11(-100, 100);
	Point2 z12(-100,-100);
	Point2 z13( 100,-100);
	Point2 z14( 100, 100);
	Point2 z21(-125, 125);
	Point2 z22(-125,-125);
	Point2 z23( 125,-125);
	Point2 z24( 125, 125);

  double sigma = 1;
  VSLAMFactor::shared_ptrK sK(new Cal3_S2(625, 625, 0, 0, 0));
  VSLAMGraph g;
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z11, sigma, 1, 1, sK)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z12, sigma, 1, 2, sK)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z13, sigma, 1, 3, sK)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z14, sigma, 1, 4, sK)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z21, sigma, 2, 1, sK)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z22, sigma, 2, 2, sK)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z23, sigma, 2, 3, sK)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z24, sigma, 2, 4, sK)));
  return g;
}

/* ************************************************************************* */
TEST( VSLAMGraph, optimizeLM)
{
  // build a graph
  VSLAMGraph graph = testGraph();
	// add 3 landmark constraints
  graph.addLandmarkConstraint(1, landmark1);
  graph.addLandmarkConstraint(2, landmark2);
  graph.addLandmarkConstraint(3, landmark3);

  // Create an initial configuration corresponding to the ground truth
  boost::shared_ptr<VSLAMConfig> initialEstimate(new VSLAMConfig);
  initialEstimate->addCameraPose(1, camera1);
  initialEstimate->addCameraPose(2, camera2);
  initialEstimate->addLandmarkPoint(1, landmark1);
  initialEstimate->addLandmarkPoint(2, landmark2);
  initialEstimate->addLandmarkPoint(3, landmark3);
  initialEstimate->addLandmarkPoint(4, landmark4);

  // Create an ordering of the variables
  list<string> keys;
  keys.push_back("l1");
  keys.push_back("l2");
  keys.push_back("l3");
  keys.push_back("l4");
  keys.push_back("x1");
  keys.push_back("x2");
  Ordering ordering(keys);

  // Create an optimizer and check its error
  // We expect the initial to be zero because config is the ground truth
  Optimizer optimizer(graph, ordering, initialEstimate, 1e-5);
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed because we started
  // with the ground truth
  Optimizer afterOneIteration = optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // check if correct
  CHECK(assert_equal(*initialEstimate,*(afterOneIteration.config())));
}


/* ************************************************************************* */
TEST( VSLAMGraph, optimizeLM2)
{
  // build a graph
  VSLAMGraph graph = testGraph();
	// add 2 camera constraints
  graph.addCameraConstraint(1, camera1);
  graph.addCameraConstraint(2, camera2);

  // Create an initial configuration corresponding to the ground truth
  boost::shared_ptr<VSLAMConfig> initialEstimate(new VSLAMConfig);
  initialEstimate->addCameraPose(1, camera1);
  initialEstimate->addCameraPose(2, camera2);
  initialEstimate->addLandmarkPoint(1, landmark1);
  initialEstimate->addLandmarkPoint(2, landmark2);
  initialEstimate->addLandmarkPoint(3, landmark3);
  initialEstimate->addLandmarkPoint(4, landmark4);

  // Create an ordering of the variables
  list<string> keys;

  keys.push_back("l1");
  keys.push_back("l2");
  keys.push_back("l3");
  keys.push_back("l4");
  keys.push_back("x1");
  keys.push_back("x2");
  Ordering ordering(keys);

  // Create an optimizer and check its error
  // We expect the initial to be zero because config is the ground truth
  Optimizer optimizer(graph, ordering, initialEstimate, 1e-5);
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed because we started
  // with the ground truth
  Optimizer afterOneIteration = optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // check if correct
  CHECK(assert_equal(*initialEstimate,*(afterOneIteration.config())));
}

/* ************************************************************************* */
int main() { TestResult tr; TestRegistry::runAllTests(tr); return 0;}
/* ************************************************************************* */
