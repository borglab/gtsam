/**
 *  @file  testPose3Pose.cpp
 *  @brief Unit tests for Pose3Factor Class
 *  @authors Frank Dellaert
 **/

#include <iostream>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include "Pose3Factor.h"
#include "LieConfig-inl.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Pose3Factor, error )
{
	// Create example
	Pose3 t1; // origin
	Pose3 t2(rodriguez(0.1,0.2,0.3),Point3(0,1,0));
	Pose3 z(rodriguez(0.2,0.2,0.3),Point3(0,1.1,0));;

	// Create factor
	Matrix measurement_covariance = eye(6);
	Pose3Factor factor("t1", "t2", z, measurement_covariance);

	// Create config
	Pose3Config x;
	x.insert("t1",t1);
	x.insert("t2",t2);

	// Get error z-h(x) -> logmap(h(x),z) = logmap(between(t1,t2),z)
	Vector actual = factor.error_vector(x);
	Vector expected = logmap(between(t1,t2),z);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
