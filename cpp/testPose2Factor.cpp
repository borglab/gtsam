/**
 *  @file  testPose2Constraint.cpp
 *  @brief Unit tests for Pose2Factor Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

/*STL/C++*/
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "NonlinearOptimizer-inl.h"
#include "NonlinearEquality.h"
#include "Pose2Factor.h"
#include "Pose2Graph.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Pose2Factor, linearize )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Matrix measurement_covariance = Matrix_(3,3,
			0.25, 0.0, 0.0,
			0.0, 0.25, 0.0,
			0.0, 0.0, 0.01
			);
	Pose2Factor constraint("p1","p2",measured, measurement_covariance);

	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4.1) looking at negative (ground truth is at 4.1,2)
	Pose2Config config;
	config.insert("p1",p1);
	config.insert("p2",p2);

	// expected linearization
	// we need the minus signs below as "inverse_square_root"
	// uses SVD and the sign is simply arbitrary (but still a square root!)
	Matrix square_root_inverse_covariance = Matrix_(3,3,
	    -2.0, 0.0, 0.0,
	    0.0, -2.0, 0.0,
	    0.0, 0.0, -10.0
	);
	Matrix expectedH1 = Matrix_(3,3,
	    0.0,-1.0,-2.1,
	    1.0, 0.0,-2.1,
	    0.0, 0.0,-1.0
	);
	Matrix expectedH2 = Matrix_(3,3,
	    1.0, 0.0, 0.0,
	    0.0, 1.0, 0.0,
	    0.0, 0.0, 1.0
	);
	GaussianFactor expected(
			"p1", square_root_inverse_covariance*expectedH1,
			"p2", square_root_inverse_covariance*expectedH2,
			Vector_(3, 0.1, -0.1, 0.0), 1.0);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> actual = constraint.linearize(config);
	CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
bool poseCompare(const std::string& key,
    const gtsam::Pose2Config& feasible,
    const gtsam::Pose2Config& input) {
  return feasible.get(key).equals(input.get(key));
}

/* ************************************************************************* */
TEST(Pose2Factor, optimize) {

	// create a Pose graph with one equality constraint and one measurement
  Pose2Graph fg;
  Pose2Config feasible;
  feasible.insert("p0", Pose2(0,0,0));
  fg.push_back(Pose2Graph::sharedFactor(
      new NonlinearEquality<Pose2Config>("p0", feasible, Pose2().dim(), poseCompare)));
  fg.push_back(Pose2Graph::sharedFactor(
      new Pose2Factor("p0", "p1", Pose2(1,2,M_PI_2), Matrix_(3,3,
          0.5, 0.0, 0.0,
          0.0, 0.5, 0.0,
          0.0, 0.0, 0.5))));

  // Create initial config
  boost::shared_ptr<Pose2Config> initial =
  		boost::shared_ptr<Pose2Config>(new Pose2Config());
  initial->insert("p0", Pose2(0,0,0));
  initial->insert("p1", Pose2(0,0,0));

  // Choose an ordering and optimize
  Ordering ordering;
  ordering += "p0","p1";
  NonlinearOptimizer<Pose2Graph, Pose2Config> optimizer(fg, ordering, initial);
  optimizer = optimizer.levenbergMarquardt(1e-15, 1e-15);

  // Check with expected config
  Pose2Config expected;
  expected.insert("p0", Pose2(0,0,0));
  expected.insert("p1", Pose2(1,2,M_PI_2));
  CHECK(assert_equal(expected, *optimizer.config()));

}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
