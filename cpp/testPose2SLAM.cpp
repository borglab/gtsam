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

#include "NonlinearOptimizer-inl.h"
#include "FactorGraph-inl.h"
#include "Ordering.h"
#include "pose2SLAM.h"

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
	Pose2Config config;
	config.insert(1,p1);
	config.insert(2,p2);
	// Linearize
	GaussianFactorGraph lfg_linearized = graph.linearize(config);
	//lfg_linearized.print("lfg_actual");

	// the expected linear factor
	GaussianFactorGraph lfg_expected;
	Matrix A1 = Matrix_(3,3,
	    0.0,-2.0, -4.2,
	    2.0, 0.0, -4.2,
	    0.0, 0.0,-10.0);

	Matrix A2 = Matrix_(3,3,
	    2.0, 0.0,  0.0,
	    0.0, 2.0,  0.0,
	    0.0, 0.0, 10.0);

	double sigma = 1;
	Vector b = Vector_(3,-0.1/sx,0.1/sy,0.0);
	sharedDiagonal probModel1 = noiseModel::Unit::Create(3);
	lfg_expected.add("x1", A1, "x2", A2, b, probModel1);

	CHECK(assert_equal(lfg_expected, lfg_linearized));
}

/* ************************************************************************* */
TEST(Pose2Graph, optimize) {

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose2Graph> fg(new Pose2Graph);
  fg->addHardConstraint(0, Pose2(0,0,0));
  fg->addConstraint(0, 1, Pose2(1,2,M_PI_2), covariance);

  // Create initial config
  boost::shared_ptr<Pose2Config> initial(new Pose2Config());
  initial->insert(0, Pose2(0,0,0));
  initial->insert(1, Pose2(0,0,0));

  // Choose an ordering and optimize
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1";
  typedef NonlinearOptimizer<Pose2Graph, Pose2Config> Optimizer;
	Optimizer::shared_solver solver(new Optimizer::solver(ordering));
  Optimizer optimizer0(fg, initial, solver);
  Optimizer::verbosityLevel verbosity = Optimizer::SILENT;
  //Optimizer::verbosityLevel verbosity = Optimizer::ERROR;
  Optimizer optimizer = optimizer0.levenbergMarquardt(1e-15, 1e-15, verbosity);

  // Check with expected config
  Pose2Config expected;
  expected.insert(0, Pose2(0,0,0));
  expected.insert(1, Pose2(1,2,M_PI_2));
  CHECK(assert_equal(expected, *optimizer.config()));
}

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose2Graph, optimizeCircle) {

	// Create a hexagon of poses
	Pose2Config hexagon = pose2SLAM::circle(6,1.0);
  Pose2 p0 = hexagon[0], p1 = hexagon[1];

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose2Graph> fg(new Pose2Graph);
  fg->addHardConstraint(0, p0);
  Pose2 delta = between(p0,p1);
  fg->addConstraint(0, 1, delta, covariance);
  fg->addConstraint(1,2, delta, covariance);
  fg->addConstraint(2,3, delta, covariance);
  fg->addConstraint(3,4, delta, covariance);
  fg->addConstraint(4,5, delta, covariance);
  fg->addConstraint(5, 0, delta, covariance);

  // Create initial config
  boost::shared_ptr<Pose2Config> initial(new Pose2Config());
  initial->insert(0, p0);
  initial->insert(1, expmap(hexagon[1],Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert(2, expmap(hexagon[2],Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert(3, expmap(hexagon[3],Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert(4, expmap(hexagon[4],Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert(5, expmap(hexagon[5],Vector_(3,-0.1, 0.1,-0.1)));

  // Choose an ordering and optimize
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1","x2","x3","x4","x5";
  typedef NonlinearOptimizer<Pose2Graph, Pose2Config> Optimizer;
	Optimizer::shared_solver solver(new Optimizer::solver(ordering));
  Optimizer optimizer0(fg, initial, solver);
  Optimizer::verbosityLevel verbosity = Optimizer::SILENT;
//  Optimizer::verbosityLevel verbosity = Optimizer::ERROR;
  Optimizer optimizer = optimizer0.levenbergMarquardt(1e-15, 1e-15, verbosity);

  Pose2Config actual = *optimizer.config();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual));

  // Check loop closure
  CHECK(assert_equal(delta,between(actual[5],actual[0])));
}

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose2Graph, findMinimumSpanningTree) {
	Pose2Graph G, T, C;
	G.addConstraint(1, 2, Pose2(0.,0.,0.), I3);
	G.addConstraint(1, 3, Pose2(0.,0.,0.), I3);
	G.addConstraint(2, 3, Pose2(0.,0.,0.), I3);

	PredecessorMap<pose2SLAM::Key> tree =
			G.findMinimumSpanningTree<pose2SLAM::Key, Pose2Factor>();
	CHECK(tree[1] == 1);
	CHECK(tree[2] == 1);
	CHECK(tree[3] == 1);
}

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose2Graph, split) {
	Pose2Graph G, T, C;
	G.addConstraint(1, 2, Pose2(0.,0.,0.), I3);
	G.addConstraint(1, 3, Pose2(0.,0.,0.), I3);
	G.addConstraint(2, 3, Pose2(0.,0.,0.), I3);

	PredecessorMap<pose2SLAM::Key> tree;
	tree.insert(1,2);
	tree.insert(2,2);
	tree.insert(3,2);

	G.split<pose2SLAM::Key, Pose2Factor>(tree, T, C);
	LONGS_EQUAL(2, T.size());
	LONGS_EQUAL(1, C.size());
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
