/**
 * @file   testPoint3.cpp
 * @brief  Unit tests for Point3 class
 */   

#include <CppUnitLite/TestHarness.h>
#include "Point3.h"

using namespace gtsam;

Point3 P(0.2,0.7,-2);

/* ************************************************************************* */
TEST( Point3, scalar_multiplication)
{
  CHECK(P*3==3*P);
}

/* ************************************************************************* */
TEST( Point3, equals)
{
  CHECK(P.equals(P));
  Point3 Q;
  CHECK(!P.equals(Q));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

