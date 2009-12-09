/**
 *  @file  testPose2Constraint.cpp
 *  @brief Unit tests for Pose2Constraint Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

/*STL/C++*/
#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include "Pose2Constraint.h"

using namespace std;
using namespace gtsam;


TEST( Pose2Constraint, constructor )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Matrix measurement_covariance = Matrix_(3,3,
			0.25, 0.0, 0.0,
			0.0, 0.25, 0.0,
			0.0, 0.0, 0.01
			);
	Pose2Constraint constraint("p1","p2",measured, measurement_covariance);

	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4) looking at negative (ground truth is at 4.1,2)
	Pose2Config config;
	config.insert(make_pair("p1",p1));
	config.insert(make_pair("p2",p2));

	// Linearize
	boost::shared_ptr<GaussianFactor> actual = constraint.linearize(config);

	// expected
	Matrix expectedH1 = Matrix_(3,3,
			0.0,-1.0,2.1,
			1.0,0.0,-2.1,
			0.0,0.0,-1.0
			);
	Matrix expectedH2 = Matrix_(3,3,
			 0.0,1.0,0.0,
			-1.0,0.0,0.0,
			 0.0,0.0,1.0
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
			Vector_(3,-0.1,-0.1,0.0), 1.0);

	CHECK(assert_equal(expected,*actual));
}
/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
