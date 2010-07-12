/**
 *  @file  testPose3Graph.cpp
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <iostream>

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost;
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

// TODO: DANGEROUS, create shared pointers
#define GTSAM_MAGIC_GAUSSIAN 6
#define GTSAM_MAGIC_KEY

#include "pose3SLAM.h"
#include "Ordering.h"

using namespace std;
using namespace gtsam;

// common measurement covariance
static Matrix covariance = eye(6);

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose3Graph, optimizeCircle) {

	// Create a hexagon of poses
	double radius = 10;
	Pose3Config hexagon = pose3SLAM::circle(6,radius);
  Pose3 gT0 = hexagon[0], gT1 = hexagon[1];

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose3Graph> fg(new Pose3Graph);
  fg->addHardConstraint(0, gT0);
  Pose3 _0T1 = between(gT0,gT1); // inv(gT0)*gT1
  double theta = M_PI/3.0;
  CHECK(assert_equal(Pose3(Rot3::yaw(-theta),Point3(radius*sin(theta),-radius*cos(theta),0)),_0T1));
  fg->addConstraint(0,1, _0T1, covariance);
  fg->addConstraint(1,2, _0T1, covariance);
  fg->addConstraint(2,3, _0T1, covariance);
  fg->addConstraint(3,4, _0T1, covariance);
  fg->addConstraint(4,5, _0T1, covariance);
  fg->addConstraint(5,0, _0T1, covariance);

  // Create initial config
  boost::shared_ptr<Pose3Config> initial(new Pose3Config());
  initial->insert(0, gT0);
  initial->insert(1, expmap(hexagon[1],Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));
  initial->insert(2, expmap(hexagon[2],Vector_(6, 0.1,-0.1, 0.1, 0.1,-0.1, 0.1)));
  initial->insert(3, expmap(hexagon[3],Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));
  initial->insert(4, expmap(hexagon[4],Vector_(6, 0.1,-0.1, 0.1, 0.1,-0.1, 0.1)));
  initial->insert(5, expmap(hexagon[5],Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));

  // Choose an ordering and optimize
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1","x2","x3","x4","x5";
  typedef NonlinearOptimizer<Pose3Graph, Pose3Config> Optimizer;
	Optimizer::shared_solver solver(new Optimizer::solver(ordering));
  Optimizer optimizer0(fg, initial, solver);
  Optimizer::verbosityLevel verbosity = Optimizer::SILENT;
//  Optimizer::verbosityLevel verbosity = Optimizer::ERROR;
  Optimizer optimizer = optimizer0.levenbergMarquardt(1e-15, 1e-15, verbosity);

  Pose3Config actual = *optimizer.config();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual,1e-4));

  // Check loop closure
  CHECK(assert_equal(_0T1,between(actual[5],actual[0]),1e-5));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
