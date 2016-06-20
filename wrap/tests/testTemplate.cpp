/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testTemplate.cpp
 * @brief  unit test for Template class
 * @author Frank Dellaert
 * @date   Dec 1, 2014
 **/

#include <wrap/Template.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace wrap;

//******************************************************************************
TEST( Template, grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in actual
  Template actual;
  TemplateGrammar g(actual);

  EXPECT(parse("template<T = {gtsam::Point2, Matrix}>", g, space_p).full);
  EXPECT_LONGS_EQUAL(2, actual.nrValues());
  EXPECT(actual.argName()=="T");
  EXPECT(actual[0]==Qualified("gtsam","Point2",Qualified::CLASS));
  EXPECT(actual[1]==Qualified("Matrix",Qualified::EIGEN));
  actual.clear();

  EXPECT(
      parse("template<ARG = {gtsam::Point2, gtsam::Point3, Vector, Matrix}>", g, space_p).full);
  EXPECT_LONGS_EQUAL(4, actual.nrValues());
  EXPECT(actual.argName()=="ARG");
  EXPECT(actual[0]==Qualified("gtsam","Point2",Qualified::CLASS));
  EXPECT(actual[1]==Qualified("gtsam","Point3",Qualified::CLASS));
  EXPECT(actual[2]==Qualified("Vector",Qualified::EIGEN));
  EXPECT(actual[3]==Qualified("Matrix",Qualified::EIGEN));
  actual.clear();

  EXPECT(parse("template<N = {1,2,3,4}>", g, space_p).full);
  EXPECT_LONGS_EQUAL(4, actual.intList().size());
  EXPECT(actual.argName()=="N");
  EXPECT_LONGS_EQUAL(1,actual.intList()[0]);
  EXPECT_LONGS_EQUAL(2,actual.intList()[1]);
  EXPECT_LONGS_EQUAL(3,actual.intList()[2]);
  EXPECT_LONGS_EQUAL(4,actual.intList()[3]);
  actual.clear();
}

//******************************************************************************
TEST( Template, CreateTemplate ) {
  boost::optional<Template> actual = //
      CreateTemplate("template<T = {gtsam::Point2, Matrix}>");
  CHECK(actual);
  // aborts if not true
  EXPECT_LONGS_EQUAL(2, actual->nrValues());
  EXPECT(actual->argName()=="T");
  EXPECT((*actual)[0]==Qualified("gtsam","Point2",Qualified::CLASS));
  EXPECT((*actual)[1]==Qualified("Matrix",Qualified::EIGEN));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
