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
// test method overloading
TEST( Class, OverloadingMethod ) {
  Class cls;
  const string name = "method1";
  EXPECT(!cls.exists(name));

  bool verbose = true, is_const = true;
  ArgumentList args;
  const ReturnValue retVal;
  const string templateArgName;
  vector<Qualified> templateArgValues;
  cls.addMethod(verbose, is_const, name, args, retVal, templateArgName,
      templateArgValues);
  EXPECT_LONGS_EQUAL(1, cls.nrMethods());
  EXPECT(cls.exists(name));
  Method& method = cls.method(name);
  EXPECT_LONGS_EQUAL(1, method.nrOverloads());

  cls.addMethod(verbose, is_const, name, args, retVal, templateArgName,
      templateArgValues);
  EXPECT_LONGS_EQUAL(1, cls.nrMethods());
  EXPECT_LONGS_EQUAL(2, method.nrOverloads());
}

/* ************************************************************************* */
// test templated methods
TEST( Class, TemplatedMethods ) {
  Class cls;
  const string name = "method";
  EXPECT(!cls.exists(name));

  bool verbose = true, is_const = true;
  ArgumentList args;
  Argument arg;
  arg.type.name = "T";
  args.push_back(arg);
  const ReturnValue retVal(ReturnType("T"));
  const string templateArgName("T");
  vector<Qualified> templateArgValues;
  templateArgValues.push_back(Qualified("Point2"));
  templateArgValues.push_back(Qualified("Point3"));
  cls.addMethod(verbose, is_const, name, args, retVal, templateArgName,
      templateArgValues);
  EXPECT_LONGS_EQUAL(2, cls.nrMethods());
  EXPECT(cls.exists(name+"Point2"));
  EXPECT(cls.exists(name+"Point3"));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
