/**
 *  @file  testUrbanOdometry.cpp
 *  @brief Unit tests for UrbanOdometry Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

/*STL/C++*/
#include <iostream>

#include <boost/assign/std/list.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include "UrbanOdometry.h"
#include "Pose2Graph.h"

using namespace std;
using namespace gtsam;


TEST( UrbanOdometry, constructor )
{
	string key1 = "x1", key2 = "x2";
	Vector measured = zero(6);
	Matrix measurement_covariance = eye(6);
	UrbanOdometry factor(key1, key2, measured, measurement_covariance);
	list<string> expected;
	expected += "x1","x2";
	CHECK(factor.keys()==expected);
}


/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
