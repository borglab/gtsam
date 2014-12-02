/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMethod.cpp
 * @brief Unit test for GlobalFunction class
 * @author Frank Dellaert
 * @date Nov 12, 2014
 **/

#include <wrap/GlobalFunction.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace wrap;

//******************************************************************************
// Constructor
TEST( GlobalFunction, Constructor ) {
  GlobalFunction f;
}

//******************************************************************************
TEST( GlobalFunction, Grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in actual
  GlobalFunctions actual;
  vector<string> namespaces;
  GlobalFunctionGrammar g(actual,namespaces);

  // a class type with namespaces
  EXPECT(parse("Vector aGlobalFunction();", g, space_p).full);
  EXPECT(parse("Vector overloadedGlobalFunction(int a);", g, space_p).full);
  EXPECT(parse("Vector overloadedGlobalFunction(int a, double b);", g, space_p).full);
  LONGS_EQUAL(2,actual.size());
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
