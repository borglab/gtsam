/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSimulated2D.cpp
 * @brief  Unit tests for simulated 2D measurement functions
 * @author Christian Potthast
 * @author Carlos Nieto
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/slam/simulated2D.h>
#include <gtsam/slam/Simulated2DValues.h>

using namespace gtsam;
using namespace std;
using namespace simulated2D;

/* ************************************************************************* */
TEST( simulated2D, Simulated2DValues )
{
	Simulated2DValues actual;
	actual.insertPose(1,Point2(1,1));
	actual.insertPoint(2,Point2(2,2));
  CHECK(assert_equal(actual,actual,1e-9));
}

/* ************************************************************************* */
TEST( simulated2D, Dprior )
{
  Point2 x(1,-9);
  Matrix numerical = numericalDerivative11(prior,x);
  Matrix computed;
  prior(x,computed);
  CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
  TEST( simulated2D, DOdo )
{
  Point2 x1(1,-9),x2(-5,6);
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
