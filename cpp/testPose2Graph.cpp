/**
 *  @file  testPose2Graph.cpp
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "NonlinearOptimizer-inl.h"
#include "NonlinearEquality.h"
#include "Ordering.h"
#include "Pose2Config.h"
#include "Pose2Graph.h"

using namespace std;
using namespace gtsam;

// common measurement covariance
static double sx=0.5, sy=0.5,st=0.1;
static Matrix covariance = Matrix_(3,3,
		sx*sx, 0.0, 0.0,
		0.0, sy*sy, 0.0,
		0.0, 0.0, st*st
		);

/* ************************************************************************* */
TEST( Pose2Graph, constructor )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Pose2Factor constraint("x1","x2",measured, covariance);
	Pose2Graph graph;
	graph.add("x1","x2",measured, covariance);
	// get the size of the graph
	size_t actual = graph.size();
	// verify
	size_t expected = 1;
	CHECK(actual == expected);

}

/* ************************************************************************* */
TEST( Pose2Graph, linerization )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Pose2Factor constraint("x1","x2",measured, covariance);
	Pose2Graph graph;
	graph.add("x1","x2",measured, covariance);

	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4) looking at negative (ground truth is at 4.1,2)
	Pose2Config config;
	config.insert("x1",p1);
	config.insert("x2",p2);
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
	lfg_expected.add("x1", A1, "x2", A2, b, sigma);

	CHECK(assert_equal(lfg_expected, lfg_linearized));
}

/* ************************************************************************* */
bool poseCompare(const std::string& key,
    const gtsam::Pose2Config& feasible,
    const gtsam::Pose2Config& input) {
  return feasible.get(key).equals(input.get(key));
}

/* ************************************************************************* */
TEST(Pose2Graph, optimize) {

	// create a Pose graph with one equality constraint and one measurement
  Pose2Graph fg;
  Pose2Config feasible;
  feasible.insert("p0", Pose2(0,0,0));
  fg.push_back(Pose2Graph::sharedFactor(
      new NonlinearEquality<Pose2Config>("p0", feasible, dim(Pose2()), poseCompare)));
  fg.add("p0", "p1", Pose2(1,2,M_PI_2), covariance);

  // Create initial config
  boost::shared_ptr<Pose2Config> initial(new Pose2Config());
  initial->insert("p0", Pose2(0,0,0));
  initial->insert("p1", Pose2(0,0,0));

  // Choose an ordering and optimize
  Ordering ordering;
  ordering += "p0","p1";
  typedef NonlinearOptimizer<Pose2Graph, Pose2Config> Optimizer;
  Optimizer optimizer(fg, ordering, initial);
  Optimizer::verbosityLevel verbosity = Optimizer::SILENT;
  //Optimizer::verbosityLevel verbosity = Optimizer::ERROR;
  optimizer = optimizer.levenbergMarquardt(1e-15, 1e-15, verbosity);

  // Check with expected config
  Pose2Config expected;
  expected.insert("p0", Pose2(0,0,0));
  expected.insert("p1", Pose2(1,2,M_PI_2));
  CHECK(assert_equal(expected, *optimizer.config()));

}

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose2Graph, optimizeCircle) {

	// Create a hexagon of poses
	Pose2Config hexagon = pose2Circle(6,1.0,'p');
  Pose2 p0 = hexagon["p0"], p1 = hexagon["p1"];

	// create a Pose graph with one equality constraint and one measurement
  Pose2Graph fg;
  Pose2Config feasible;
  feasible.insert("p0", p0);
  fg.push_back(Pose2Graph::sharedFactor(
      new NonlinearEquality<Pose2Config>("p0", feasible, dim(Pose2()), poseCompare)));
  Pose2 delta = between(p0,p1);
  fg.add("p0", "p1", delta, covariance);
  fg.add("p1", "p2", delta, covariance);
  fg.add("p2", "p3", delta, covariance);
  fg.add("p3", "p4", delta, covariance);
  fg.add("p4", "p5", delta, covariance);
  fg.add("p5", "p0", delta, covariance);

  // Create initial config
  boost::shared_ptr<Pose2Config> initial(new Pose2Config());
  initial->insert("p0", p0);
  initial->insert("p1", expmap(hexagon["p1"],Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert("p2", expmap(hexagon["p2"],Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert("p3", expmap(hexagon["p3"],Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert("p4", expmap(hexagon["p4"],Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert("p5", expmap(hexagon["p5"],Vector_(3,-0.1, 0.1,-0.1)));

  // Choose an ordering and optimize
  Ordering ordering;
  ordering += "p0","p1","p2","p3","p4","p5";
  typedef NonlinearOptimizer<Pose2Graph, Pose2Config> Optimizer;
  Optimizer optimizer(fg, ordering, initial);
  Optimizer::verbosityLevel verbosity = Optimizer::SILENT;
//  Optimizer::verbosityLevel verbosity = Optimizer::ERROR;
  optimizer = optimizer.levenbergMarquardt(1e-15, 1e-15, verbosity);

  Pose2Config actual = *optimizer.config();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual));

  // Check loop closure
  CHECK(assert_equal(delta,between(actual["p5"],actual["p0"])));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
