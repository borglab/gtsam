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
Point3 landmark1_local(-1.0,-1.0, 0.0);
Point3 landmark2_local(-1.0, 1.0, 0.0);
Point3 landmark3_local( 1.0, 1.0, 0.0);
Point3 landmark4_local( 1.0,-1.0, 0.0);

Pose3 camera1_local(Matrix_(3,3,
		       1., 0., 0.,
		       0.,-1., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,6.25));

Pose3 camera2_local(Matrix_(3,3,
		       1., 0., 0.,
		       0.,-1., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,5.00));

Point3 landmark1() { return landmark1_local;}
Point3 landmark2() { return landmark2_local;}
Point3 landmark3() { return landmark3_local;}
Point3 landmark4() { return landmark4_local;}

Pose3 camera1() { return camera1_local;}
Pose3 camera2() { return camera2_local;}


/* ************************************************************************* */
VSLAMGraph testGraph() {
  Point2 z11(-100, 100);
	Point2 z12(-100, -100);
	Point2 z13(100, -100);
	Point2 z14(100, 100);
	Point2 z21(-125, 125);
	Point2 z22(-125, -125);
	Point2 z23(125, -125);
	Point2 z24(125, 125);

  double sigma = 1;
  Cal3_S2 K(625, 625, 0, 0, 0);
  VSLAMGraph g;
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z11.vector(), sigma, 1, 1, K)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z12.vector(), sigma, 1, 2, K)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z13.vector(), sigma, 1, 3, K)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z14.vector(), sigma, 1, 4, K)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z21.vector(), sigma, 2, 1, K)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z22.vector(), sigma, 2, 2, K)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z23.vector(), sigma, 2, 3, K)));
  g.push_back(VSLAMFactor::shared_ptr(new VSLAMFactor(z24.vector(), sigma, 2, 4, K)));

  return g;
}

/* ************************************************************************* */
TEST( VSLAMGraph, optimizeLM)
{
  // build a graph
  VSLAMGraph graph = testGraph();

  // Create a configuration corresponding to the ground truth
  VSLAMConfig groundTruth;
  groundTruth.addCameraPose(1, camera1());
  groundTruth.addCameraPose(2, camera2());
  groundTruth.addLandmarkPoint(1, landmark1());
  groundTruth.addLandmarkPoint(2, landmark2());
  groundTruth.addLandmarkPoint(3, landmark3());
  groundTruth.addLandmarkPoint(4, landmark4());

  // Create an ordering of the variables
  list<string> keys;
  keys.push_back("l1");
  keys.push_back("l2");
  keys.push_back("l3");
  keys.push_back("l4");
  keys.push_back("x1");
  keys.push_back("x2");
  Ordering ordering(keys);

  // optimize it
  double relativeErrorTreshold=1e-6, absoluteErrorTreshold=1.0;
  int verbosity=0;

  Optimizer::shared_config initialConfig(new VSLAMConfig(groundTruth));
  Optimizer optimizer(graph, ordering, initialConfig, 1e-5);

  // We expect the initial to be zero because config is the ground truth
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed because we started
  // with the ground truth
  Optimizer afterOneIteration = optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  //afterOneIteration.config()->print();

  // check if correct
  CHECK(groundTruth.equals(*(afterOneIteration.config())) );
}

/* ************************************************************************* */
int main() { TestResult tr; TestRegistry::runAllTests(tr); return 0;}
/* ************************************************************************* */
