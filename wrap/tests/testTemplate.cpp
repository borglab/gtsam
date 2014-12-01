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
  EXPECT_LONGS_EQUAL(2, actual.argValues.size());
  EXPECT(actual.argName=="T");
  EXPECT(actual.argValues[0]==Qualified("gtsam","Point2",Qualified::CLASS));
  EXPECT(actual.argValues[1]==Qualified("Matrix",Qualified::EIGEN));
  actual.clear();

  EXPECT(parse("template<ARG = {gtsam::Point2, gtsam::Point3, Vector, Matrix}>", g, space_p).full);
  EXPECT_LONGS_EQUAL(4, actual.argValues.size());
  EXPECT(actual.argName=="ARG");
  EXPECT(actual.argValues[0]==Qualified("gtsam","Point2",Qualified::CLASS));
  EXPECT(actual.argValues[1]==Qualified("gtsam","Point3",Qualified::CLASS));
  EXPECT(actual.argValues[2]==Qualified("Vector",Qualified::EIGEN));
  EXPECT(actual.argValues[3]==Qualified("Matrix",Qualified::EIGEN));
  actual.clear();
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
