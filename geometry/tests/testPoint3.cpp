/**
 * @file   testPoint3.cpp
 * @brief  Unit tests for Point3 class
 */   

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Point3.h>

using namespace gtsam;

Point3 P(0.2,0.7,-2);

/* ************************************************************************* */
TEST( Point3, arithmetic)
{
  CHECK(P*3==3*P);
	CHECK(assert_equal( Point3(-1,-5,-6), -Point3(1,5,6) ));
	CHECK(assert_equal( Point3(2,5,6), Point3(1,4,5)+Point3(1,1,1)));
	CHECK(assert_equal( Point3(0,3,4), Point3(1,4,5)-Point3(1,1,1)));
	CHECK(assert_equal( Point3(2,8,6), Point3(1,4,3)*2));
	CHECK(assert_equal( Point3(2,2,6), 2*Point3(1,1,3)));
	CHECK(assert_equal( Point3(1,2,3), Point3(2,4,6)/2));
}

/* ************************************************************************* */
TEST( Point3, equals)
{
  CHECK(P.equals(P));
  Point3 Q;
  CHECK(!P.equals(Q));
}

/* ************************************************************************* */
TEST( Point3, dot)
{
	CHECK(Point3::dot(Point3(0,0,0),Point3(1,1,0)) == 0);
	CHECK(Point3::dot(Point3(1,1,1),Point3(1,1,0)) == 2);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

