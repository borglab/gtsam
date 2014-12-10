/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMethod.cpp
 * @brief Unit test for Method class
 * @author Frank Dellaert
 * @date Nov 12, 2014
 **/

#include <wrap/Method.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace wrap;

//******************************************************************************
// Constructor
TEST( Method, Constructor ) {
  Method method;
}

//******************************************************************************
// addOverload
TEST( Method, addOverload ) {
  Method method;
  ArgumentList args;
  bool is_const = true;
  const ReturnValue retVal1(ReturnType("return_type1"));
  method.addOverload("myName", args, retVal1, is_const);
  const ReturnValue retVal2(ReturnType("return_type2"));
  method.addOverload("myName", args, retVal2, is_const);
  EXPECT_LONGS_EQUAL(2, method.nrOverloads());
}

////******************************************************************************
//TEST( Method, grammar ) {
//
//  using classic::space_p;
//
//  // Create type grammar that will place result in actual
//  Method actual;
//  method_grammar method_g(actual);
//
//  // a class type with namespaces
//  EXPECT(parse("double x() const;", method_g, space_p).full);
//}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
