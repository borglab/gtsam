/**
 *  @file  testPose3Graph.cpp
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <iostream>

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "NonlinearOptimizer-inl.h"
#include "NonlinearEquality.h"
#include "Ordering.h"
#include "Pose3Graph.h"

using namespace std;
using namespace gtsam;

// common measurement covariance
static Matrix covariance = eye(6);

/* ************************************************************************* */
bool poseCompare(const std::string& key,
    const gtsam::Pose3Config& feasible,
    const gtsam::Pose3Config& input) {
  return feasible.get(key).equals(input.get(key));
}

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose3Graph, optimizeCircle) {

	// Create a hexagon of poses
	Pose3Config hexagon = pose3Circle(6,1.0,'p');
  Pose3 p0 = hexagon["p0"], p1 = hexagon["p1"];

	// create a Pose graph with one equality constraint and one measurement
  Pose3Graph fg;
  Pose3Config feasible;
  feasible.insert("p0", p0);
  fg.push_back(Pose3Graph::sharedFactor(
      new NonlinearEquality<Pose3Config>("p0", feasible, dim(Pose3()), poseCompare)));
  Pose3 delta = between(p0,p1);
  fg.add("p0", "p1", delta, covariance);
  fg.add("p1", "p2", delta, covariance);
  fg.add("p2", "p3", delta, covariance);
  fg.add("p3", "p4", delta, covariance);
  fg.add("p4", "p5", delta, covariance);
  fg.add("p5", "p0", delta, covariance);

  // Create initial config
  boost::shared_ptr<Pose3Config> initial(new Pose3Config());
  initial->insert("p0", p0);
  initial->insert("p1", expmap(hexagon["p1"],Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));
  initial->insert("p2", expmap(hexagon["p2"],Vector_(6, 0.1,-0.1, 0.1, 0.1,-0.1, 0.1)));
  initial->insert("p3", expmap(hexagon["p3"],Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));
  initial->insert("p4", expmap(hexagon["p4"],Vector_(6, 0.1,-0.1, 0.1, 0.1,-0.1, 0.1)));
  initial->insert("p5", expmap(hexagon["p5"],Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));

  // Choose an ordering and optimize
  Ordering ordering;
  ordering += "p0","p1","p2","p3","p4","p5";
  typedef NonlinearOptimizer<Pose3Graph, Pose3Config> Optimizer;
  Optimizer optimizer(fg, ordering, initial);
  Optimizer::verbosityLevel verbosity = Optimizer::SILENT;
//  Optimizer::verbosityLevel verbosity = Optimizer::ERROR;
  optimizer = optimizer.levenbergMarquardt(1e-15, 1e-15, verbosity);

  Pose3Config actual = *optimizer.config();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual,1e-5));

  // Check loop closure
  CHECK(assert_equal(delta,between(actual["p5"],actual["p0"]),1e-5));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
