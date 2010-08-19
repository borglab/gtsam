/**
 * @file   testSimulated3D.cpp
 * @brief  Unit tests for simulated 3D measurement functions
 * @author Alex Cunningham
 **/

#include <iostream>
#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/slam/Simulated3D.h>

using namespace gtsam;
using namespace simulated3D;

/* ************************************************************************* */
TEST( simulated3D, Dprior )
{
	Pose3 x1(rodriguez(0, 0, 1.57), Point3(1, 5, 0));
	Vector v = logmap(x1);
	Matrix numerical = numericalDerivative11(prior,v);
	Matrix computed = Dprior(v);
	CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
TEST( simulated3D, DOdo1 )
{
	Pose3 x1(rodriguez(0, 0, 1.57), Point3(1, 5, 0));
	Vector v1 = logmap(x1);
	Pose3 x2(rodriguez(0, 0, 0), Point3(2, 3, 0));
	Vector v2 = logmap(x2);
	Matrix numerical = numericalDerivative21(odo,v1,v2);
	Matrix computed = Dodo1(v1,v2);
	CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
TEST( simulated3D, DOdo2 )
{
	Pose3 x1(rodriguez(0, 0, 1.57), Point3(1, 5, 0));
	Vector v1 = logmap(x1);
	Pose3 x2(rodriguez(0, 0, 0), Point3(2, 3, 0));
	Vector v2 = logmap(x2);
	Matrix numerical = numericalDerivative22(odo,v1,v2);
	Matrix computed = Dodo2(v1,v2);
	CHECK(assert_equal(numerical,computed,1e-9));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
