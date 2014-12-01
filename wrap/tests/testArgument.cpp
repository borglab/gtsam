/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testArgument.cpp
 * @brief Unit test for Argument class
 * @author Frank Dellaert
 * @date Nov 12, 2014
 **/

#include <wrap/Argument.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace wrap;

//******************************************************************************
TEST( Argument, grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in actual
  Argument actual, arg0;
  ArgumentGrammar g(actual);

  EXPECT(parse("const gtsam::Point2& p4", g, space_p).full);
  EXPECT(actual.type==Qualified("gtsam","Point2",Qualified::CLASS));
  EXPECT(actual.name=="p4");
  EXPECT(actual.is_const);
  EXPECT(actual.is_ref);
  EXPECT(!actual.is_ptr);
  actual = arg0;

  EXPECT(parse("Point2& p", g, space_p).full);
  EXPECT(actual.type==Qualified("Point2",Qualified::CLASS));
  EXPECT(actual.name=="p");
  EXPECT(!actual.is_const);
  EXPECT(actual.is_ref);
  EXPECT(!actual.is_ptr);
  actual = arg0;

  EXPECT(parse("gtsam::Point2* p3", g, space_p).full);
  EXPECT(actual.type==Qualified("gtsam","Point2",Qualified::CLASS));
  EXPECT(actual.name=="p3");
  EXPECT(!actual.is_const);
  EXPECT(!actual.is_ref);
  EXPECT(actual.is_ptr);
  actual = arg0;

  EXPECT(parse("char a", g, space_p).full);
  EXPECT(actual==Argument(Qualified("char",Qualified::BASIS),"a"));
  actual = arg0;

  EXPECT(parse("unsigned char a", g, space_p).full);
  EXPECT(actual==Argument(Qualified("unsigned char",Qualified::BASIS),"a"));
  actual = arg0;

  EXPECT(parse("Vector v", g, space_p).full);
  EXPECT(actual==Argument(Qualified("Vector",Qualified::EIGEN),"v"));
  actual = arg0;

  EXPECT(parse("const Matrix& m", g, space_p).full);
  EXPECT(actual.type==Qualified("Matrix",Qualified::EIGEN));
  EXPECT(actual.name=="m");
  EXPECT(actual.is_const);
  EXPECT(actual.is_ref);
  EXPECT(!actual.is_ptr);
  actual = arg0;
}

//******************************************************************************
TEST( ArgumentList, grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in actual
  ArgumentList actual;
  ArgumentListGrammar g(actual);

  EXPECT(parse("(const gtsam::Point2& p4)", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();

  EXPECT(parse("()", g, space_p).full);
  EXPECT_LONGS_EQUAL(0, actual.size());
  actual.clear();

  EXPECT(parse("(char a)", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();

  EXPECT(parse("(unsigned char a)", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();

  EXPECT(parse("(Vector v, Matrix m)", g, space_p).full);
  EXPECT_LONGS_EQUAL(2, actual.size());
  EXPECT(actual[0]==Argument(Qualified("Vector",Qualified::EIGEN),"v"));
  EXPECT(actual[1]==Argument(Qualified("Matrix",Qualified::EIGEN),"m"));
  actual.clear();

  EXPECT(parse("(Point2 p)", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();

  EXPECT(parse("(Point2 p1, Point3 p2)", g, space_p).full);
  EXPECT_LONGS_EQUAL(2, actual.size());
  EXPECT(actual[0]==Argument(Qualified("Point2"),"p1"));
  EXPECT(actual[1]==Argument(Qualified("Point3"),"p2"));
  actual.clear();

  EXPECT(parse("(gtsam::Point2 p3)", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
