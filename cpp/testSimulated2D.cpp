/**
 * @file   testSimulated2D.cpp
 * @brief  Unit tests for simulated 2D measurement functions
 * @author Christian Potthast
 * @author Carlos Nieto
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include "numericalDerivative.h"
#include "simulated2D.h"

using namespace gtsam;
using namespace std;
using namespace simulated2D;

/* ************************************************************************* */
TEST( simulated2D, Dprior )
{
  Vector x(2);x(0)=1;x(1)=-9;
  Matrix numerical = numericalDerivative11(prior,x);
  Matrix computed;
  prior(x,computed);
  CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
  TEST( simulated2D, DOdo )
{
  Vector x1(2);x1(0)= 1;x1(1)=-9;
  Vector x2(2);x2(0)=-5;x2(1)= 6;
  Matrix H1,H2;
  odo(x1,x2,H1,H2);
  Matrix A1 = numericalDerivative21(odo,x1,x2);
  CHECK(assert_equal(A1,H1,1e-9));
  Matrix A2 = numericalDerivative22(odo,x1,x2);
  CHECK(assert_equal(A2,H2,1e-9));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
