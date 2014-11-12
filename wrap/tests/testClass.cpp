/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testClass.cpp
 * @brief Unit test for Class class
 * @author Frank Dellaert
 * @date Nov 12, 2014
 **/

#include <wrap/Class.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
// Constructor
TEST( Class, Constructor ) {
  Class cls;
}

/* ************************************************************************* */
// addMethodOverloads
TEST( Class, addMethod ) {
  Class cls;
  const string name = "method1";
  EXPECT(!cls.exists(name));

  bool verbose=true, is_const=true;
  ArgumentList args;
  const ReturnValue retVal;
  const string templateArgName;
  vector<Qualified> templateArgValues;
  cls.addMethod(verbose, is_const, name, args, retVal, templateArgName,
      templateArgValues);
  EXPECT_LONGS_EQUAL(1,cls.nrMethods());
  EXPECT(cls.exists(name));
  Method& method = cls.method(name);
  EXPECT_LONGS_EQUAL(1,method.returnVals.size());

  cls.addMethod(verbose, is_const, name, args, retVal, templateArgName,
      templateArgValues);
  EXPECT_LONGS_EQUAL(1,cls.nrMethods());
  EXPECT_LONGS_EQUAL(2,method.returnVals.size());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
