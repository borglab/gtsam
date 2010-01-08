/**
 * @file   testPoint2.cpp
 * @brief  Unit tests for Point2 class
 * @author Frank Dellaert
 **/

#include <CppUnitLite/TestHarness.h>
#include "Point2.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Point2, expmap) {
  Vector d(2);d(0)=1;d(1)=-1;
  Point2 a(4,5), b=expmap(a,d),c(5,4);
  CHECK(assert_equal(b,c));
}

/* ************************************************************************* */
TEST( Point2, add) {
  CHECK(assert_equal( Point2(4,5)+Point2(1,1), Point2(5,6) ));
}

/* ************************************************************************* */
TEST( Point2, subtract) {
  CHECK(assert_equal( Point2(4,5)-Point2(1,1), Point2(3,4) ));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
