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
  const Template tmplate;
  cls.addMethod(verbose, is_const, name, args, retVal, tmplate);
  EXPECT_LONGS_EQUAL(1, cls.nrMethods());
  EXPECT(cls.exists(name));
  EXPECT_LONGS_EQUAL(1, cls.method(name).nrOverloads());

  // add an overload w different argument list
  args.push_back(Argument(Qualified("Vector",Qualified::EIGEN),"v"));
  cls.addMethod(verbose, is_const, name, args, retVal, tmplate);
  EXPECT_LONGS_EQUAL(1, cls.nrMethods());
  EXPECT_LONGS_EQUAL(2, cls.method(name).nrOverloads());

  // add with non-trivial template list, will create two different functions
  boost::optional<Template> t = //
      CreateTemplate("template<T = {Point2, Point3}>");
  CHECK(t);
  cls.addMethod(verbose, is_const, name, args, retVal, *t);
  EXPECT_LONGS_EQUAL(3, cls.nrMethods());
  EXPECT_LONGS_EQUAL(2, cls.method(name).nrOverloads());
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
  arg.type.name_ = "T";
  args.push_back(arg);
  const ReturnValue retVal(ReturnType("T"));
  boost::optional<Template> tmplate = //
      CreateTemplate("template<T = {Point2, Point3}>");
  CHECK(tmplate);
  cls.addMethod(verbose, is_const, name, args, retVal, *tmplate);
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
