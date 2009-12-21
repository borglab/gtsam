/**
 *  @file  testPose2Constraint.cpp
 *  @brief Unit tests for Pose2Factor Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

/*STL/C++*/
#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include <Boost/shared_ptr.hpp>
#include "NonlinearOptimizer-inl.h"
#include "NonlinearEquality.h"
#include "Pose2Factor.h"
#include "Pose2Graph.h"

using namespace std;
using namespace gtsam;
using namespace boost;

TEST( Pose2Factor, constructor )
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

	// Linearize
	boost::shared_ptr<GaussianFactor> actual = constraint.linearize(config);

	// expected
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
	// we need the minus signs as inverse_square_root uses SVD and sign is simply arbitrary (still a ssquare root!)
	Matrix square_root_inverse_covariance = Matrix_(3,3,
	    -2.0, 0.0, 0.0,
	    0.0, -2.0, 0.0,
	    0.0, 0.0, -10.0
	);
	GaussianFactor expected(
			"p1", square_root_inverse_covariance*expectedH1,
			"p2", square_root_inverse_covariance*expectedH2,
			Vector_(3, 0.1, -0.1, 0.0), 1.0);

	CHECK(assert_equal(expected,*actual));
}

bool poseCompare(const std::string& key,
    const gtsam::Pose2Config& feasible,
    const gtsam::Pose2Config& input) {
  return feasible.get(key).equals(input.get(key));
}

TEST(Pose2Factor, optimize) {
  Pose2Graph fg;
  shared_ptr<Pose2Config> config = shared_ptr<Pose2Config>(new Pose2Config());
  Pose2Config feasible;
  feasible.insert("p0", Pose2(0,0,0));
  fg.push_back(Pose2Graph::sharedFactor(
      new NonlinearEquality<Pose2Config>("p0", feasible, Pose2().dim(), poseCompare)));
  config->insert("p0", Pose2(0,0,0));
  fg.push_back(Pose2Graph::sharedFactor(
      new Pose2Factor("p0", "p1", Pose2(1,2,M_PI_2), Matrix_(3,3,
          0.5, 0.0, 0.0,
          0.0, 0.5, 0.0,
          0.0, 0.0, 0.5))));
  config->insert("p1", Pose2(0,0,0));
  Ordering ordering;
  ordering.push_back("p0");
  ordering.push_back("p1");
  NonlinearOptimizer<Pose2Graph, Pose2Config> optimizer(fg, ordering, config);
  optimizer = optimizer.levenbergMarquardt(1e-10, 1e-10);
  Pose2 actual0 = optimizer.config()->get("p0");
  Pose2 actual1 = optimizer.config()->get("p1");
  Pose2 expected0 = Pose2(0,0,0);
  Pose2 expected1 = Pose2(1,2,M_PI_2);
  CHECK(assert_equal(expected0, actual0));
  CHECK(assert_equal(expected0, actual0));
}


/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
