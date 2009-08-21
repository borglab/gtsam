/**
 * @file   testSimulated3D.cpp
 * @brief  Unit tests for simulated 3D measurement functions
 * @author Alex Cunningham
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include "Pose3.h"
#include "numericalDerivative.h"
#include "Simulated3D.h"

using namespace gtsam;

/* ************************************************************************* */
TEST( simulated3D, Dprior_3D )
{
	Pose3 x1(rodriguez(0, 0, 1.57), Point3(1, 5, 0));
	Vector v = x1.vector();
	Matrix numerical = NumericalDerivative11(prior_3D,v);
	Matrix computed = Dprior_3D(v);
	CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
TEST( simulated3D, DOdo1 )
{
	Pose3 x1(rodriguez(0, 0, 1.57), Point3(1, 5, 0));
	Vector v1 = x1.vector();
	Pose3 x2(rodriguez(0, 0, 0), Point3(2, 3, 0));
	Vector v2 = x2.vector();
	Matrix numerical = NumericalDerivative21(odo_3D,v1,v2);
	Matrix computed = Dodo1_3D(v1,v2);
	CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
TEST( simulated3D, DOdo2 )
{
	Pose3 x1(rodriguez(0, 0, 1.57), Point3(1, 5, 0));
	Vector v1 = x1.vector();
	Pose3 x2(rodriguez(0, 0, 0), Point3(2, 3, 0));
	Vector v2 = x2.vector();
	Matrix numerical = NumericalDerivative22(odo_3D,v1,v2);
	Matrix computed = Dodo2_3D(v1,v2);
	CHECK(assert_equal(numerical,computed,1e-9));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
