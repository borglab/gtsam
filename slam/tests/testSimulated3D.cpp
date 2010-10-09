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
TEST( simulated3D, Values )
{
	Values actual;
	actual.insert(PointKey(1),Point3(1,1,1));
	actual.insert(PoseKey(2),Point3(2,2,2));
	EXPECT(assert_equal(actual,actual,1e-9));
}

/* ************************************************************************* */
TEST( simulated3D, Dprior )
{
	Point3 x(1,-9, 7);
	Matrix numerical = numericalDerivative11<Point3, Point3>(boost::bind(prior, _1, boost::none),x);
	Matrix computed;
	prior(x,computed);
	EXPECT(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
TEST( simulated3D, DOdo )
{
	Point3 x1(1,-9,7),x2(-5,6,7);
	Matrix H1,H2;
	odo(x1,x2,H1,H2);
	Matrix A1 = numericalDerivative21<Point3, Point3, Point3>(boost::bind(odo, _1, _2, boost::none, boost::none),x1,x2);
	EXPECT(assert_equal(A1,H1,1e-9));
	Matrix A2 = numericalDerivative22<Point3, Point3, Point3>(boost::bind(odo, _1, _2, boost::none, boost::none),x1,x2);
	EXPECT(assert_equal(A2,H2,1e-9));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
