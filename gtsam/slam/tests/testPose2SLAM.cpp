/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testPose2Graph.cpp
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost;
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/slam/pose2SLAM.h>
//#include <gtsam/slam/Pose2SLAMOptimizer.h>

using namespace std;
using namespace gtsam;

// common measurement covariance
static double sx=0.5, sy=0.5,st=0.1;
static noiseModel::Gaussian::shared_ptr covariance(
		noiseModel::Gaussian::Covariance(Matrix_(3, 3,
	sx*sx, 0.0, 0.0,
	0.0, sy*sy, 0.0,
	0.0, 0.0, st*st
	))), I3(noiseModel::Unit::Create(3));

/* ************************************************************************* */
TEST( Pose2Graph, constructor )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Pose2Factor constraint(1,2,measured, covariance);
	Pose2Graph graph;
	graph.addConstraint(1,2,measured, covariance);
	// get the size of the graph
	size_t actual = graph.size();
	// verify
	size_t expected = 1;
	CHECK(actual == expected);

}

/* ************************************************************************* */
TEST( Pose2Graph, linearization )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Pose2Factor constraint(1,2,measured, covariance);
	Pose2Graph graph;
	graph.addConstraint(1,2,measured, covariance);

	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4) looking at negative (ground truth is at 4.1,2)
	Pose2Values config;
	config.insert(1,p1);
	config.insert(2,p2);
	// Linearize
	Ordering ordering(*config.orderingArbitrary());
	boost::shared_ptr<FactorGraph<GaussianFactor> > lfg_linearized = graph.linearize(config, ordering);
	//lfg_linearized->print("lfg_actual");

	// the expected linear factor
	FactorGraph<GaussianFactor> lfg_expected;
	Matrix A1 = Matrix_(3,3,
	    0.0,-2.0, -4.2,
	    2.0, 0.0, -4.2,
	    0.0, 0.0,-10.0);

	Matrix A2 = Matrix_(3,3,
	    2.0, 0.0,  0.0,
	    0.0, 2.0,  0.0,
	    0.0, 0.0, 10.0);

	Vector b = Vector_(3,-0.1/sx,0.1/sy,0.0);
	SharedDiagonal probModel1 = noiseModel::Unit::Create(3);
	lfg_expected.push_back(JacobianFactor::shared_ptr(new JacobianFactor(ordering["x1"], A1, ordering["x2"], A2, b, probModel1)));

	CHECK(assert_equal(lfg_expected, *lfg_linearized));
}

/* ************************************************************************* */
TEST(Pose2Graph, optimize) {

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose2Graph> fg(new Pose2Graph);
  fg->addHardConstraint(0, Pose2(0,0,0));
  fg->addConstraint(0, 1, Pose2(1,2,M_PI_2), covariance);

  // Create initial config
  boost::shared_ptr<Pose2Values> initial(new Pose2Values());
  initial->insert(0, Pose2(0,0,0));
  initial->insert(1, Pose2(0,0,0));

  // Choose an ordering and optimize
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1";
  typedef NonlinearOptimizer<Pose2Graph, Pose2Values> Optimizer;

  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-15, 1e-15);
  Optimizer optimizer0(fg, initial, ordering, params);
  Optimizer optimizer = optimizer0.levenbergMarquardt();

  // Check with expected config
  Pose2Values expected;
  expected.insert(0, Pose2(0,0,0));
  expected.insert(1, Pose2(1,2,M_PI_2));
  CHECK(assert_equal(expected, *optimizer.values()));
}

/* ************************************************************************* */
// test optimization with 3 poses
TEST(Pose2Graph, optimizeThreePoses) {

	// Create a hexagon of poses
	Pose2Values hexagon = pose2SLAM::circle(3,1.0);
  Pose2 p0 = hexagon[0], p1 = hexagon[1];

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose2Graph> fg(new Pose2Graph);
  fg->addHardConstraint(0, p0);
  Pose2 delta = p0.between(p1);
  fg->addConstraint(0, 1, delta, covariance);
  fg->addConstraint(1, 2, delta, covariance);
  fg->addConstraint(2, 0, delta, covariance);

  // Create initial config
  boost::shared_ptr<Pose2Values> initial(new Pose2Values());
  initial->insert(0, p0);
  initial->insert(1, hexagon[1].expmap(Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert(2, hexagon[2].expmap(Vector_(3, 0.1,-0.1, 0.1)));

  // Choose an ordering
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1","x2";

  // optimize
  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-15, 1e-15);
  pose2SLAM::Optimizer optimizer0(fg, initial, ordering, params);
  pose2SLAM::Optimizer optimizer = optimizer0.levenbergMarquardt();

  Pose2Values actual = *optimizer.values();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual));
}

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose2Graph, optimizeCircle) {

	// Create a hexagon of poses
	Pose2Values hexagon = pose2SLAM::circle(6,1.0);
  Pose2 p0 = hexagon[0], p1 = hexagon[1];

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose2Graph> fg(new Pose2Graph);
  fg->addHardConstraint(0, p0);
  Pose2 delta = p0.between(p1);
  fg->addConstraint(0, 1, delta, covariance);
  fg->addConstraint(1,2, delta, covariance);
  fg->addConstraint(2,3, delta, covariance);
  fg->addConstraint(3,4, delta, covariance);
  fg->addConstraint(4,5, delta, covariance);
  fg->addConstraint(5, 0, delta, covariance);

  // Create initial config
  boost::shared_ptr<Pose2Values> initial(new Pose2Values());
  initial->insert(0, p0);
  initial->insert(1, hexagon[1].expmap(Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert(2, hexagon[2].expmap(Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert(3, hexagon[3].expmap(Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert(4, hexagon[4].expmap(Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert(5, hexagon[5].expmap(Vector_(3,-0.1, 0.1,-0.1)));

  // Choose an ordering
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1","x2","x3","x4","x5";

  // optimize
  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-15, 1e-15);
  pose2SLAM::Optimizer optimizer0(fg, initial, ordering, params);
  pose2SLAM::Optimizer optimizer = optimizer0.levenbergMarquardt();

  Pose2Values actual = *optimizer.values();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual));

  // Check loop closure
  CHECK(assert_equal(delta,actual[5].between(actual[0])));

//  Pose2SLAMOptimizer myOptimizer("3");

//  Matrix A1 = myOptimizer.a1();
//  LONGS_EQUAL(3,  A1.size1());
//  LONGS_EQUAL(17, A1.size2()); // 7 + 7 + 3
//
//  Matrix A2 = myOptimizer.a2();
//  LONGS_EQUAL(3, A1.size1());
//  LONGS_EQUAL(7, A2.size2()); // 7
//
//  Vector b1 = myOptimizer.b1();
//  LONGS_EQUAL(9, b1.size()); // 3 + 3 + 3
//
//  Vector b2 = myOptimizer.b2();
//  LONGS_EQUAL(3, b2.size()); // 3
//
//  // Here, call matlab to
//  // A=[A1;A2], b=[b1;b2]
//  // R=qr(A1)
//  // call pcg on A,b, with preconditioner R -> get x
//
//  Vector x = myOptimizer.optimize();
//  LONGS_EQUAL(9, x.size()); // 3 + 3 + 3
//
//  myOptimizer.update(x);
//
//  Pose2Values expected;
//  expected.insert(0, Pose2(0.,0.,0.));
//  expected.insert(1, Pose2(1.,0.,0.));
//  expected.insert(2, Pose2(2.,0.,0.));
//
//  // Check with ground truth
//  CHECK(assert_equal(expected, *myOptimizer.theta()));
}

/* ************************************************************************* */
TEST(Pose2Graph, optimize2) {
//  Pose2SLAMOptimizer myOptimizer("100");
//  Matrix A1 = myOptimizer.a1();
//  Matrix A2 = myOptimizer.a2();
//  cout << "A1: " << A1.size1() << " " << A1.size2() << endl;
//  cout << "A2: " << A2.size1() << " " << A2.size2() << endl;
//
//  //cout << "error: " << myOptimizer.error() << endl;
//  for(int i = 0; i<10; i++) {
//  	myOptimizer.linearize();
//  	Vector x = myOptimizer.optimize();
//  	myOptimizer.update(x);
//  }
//  //cout << "error: " << myOptimizer.error() << endl;
//  CHECK(myOptimizer.error() < 1.);
}

///* ************************************************************************* */
// SL-NEEDED? TEST(Pose2Graph, findMinimumSpanningTree) {
//	Pose2Graph G, T, C;
//	G.addConstraint(1, 2, Pose2(0.,0.,0.), I3);
//	G.addConstraint(1, 3, Pose2(0.,0.,0.), I3);
//	G.addConstraint(2, 3, Pose2(0.,0.,0.), I3);
//
//	PredecessorMap<pose2SLAM::Key> tree =
//			G.findMinimumSpanningTree<pose2SLAM::Key, Pose2Factor>();
//	CHECK(tree[1] == 1);
//	CHECK(tree[2] == 1);
//	CHECK(tree[3] == 1);
//}
//
///* ************************************************************************* */
// SL-NEEDED? TEST(Pose2Graph, split) {
//	Pose2Graph G, T, C;
//	G.addConstraint(1, 2, Pose2(0.,0.,0.), I3);
//	G.addConstraint(1, 3, Pose2(0.,0.,0.), I3);
//	G.addConstraint(2, 3, Pose2(0.,0.,0.), I3);
//
//	PredecessorMap<pose2SLAM::Key> tree;
//	tree.insert(1,2);
//	tree.insert(2,2);
//	tree.insert(3,2);
//
//	G.split<pose2SLAM::Key, Pose2Factor>(tree, T, C);
//	LONGS_EQUAL(2, T.size());
//	LONGS_EQUAL(1, C.size());
//}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
