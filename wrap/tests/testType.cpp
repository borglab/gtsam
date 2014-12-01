/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testType.cpp
 * @brief  unit test for parsing a fully qualified type
 * @author Frank Dellaert
 * @date   Nov 30, 2014
 **/

#include <wrap/Qualified.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace wrap;

//******************************************************************************
TEST( Type, Constructor1 ) {
  Qualified actual("Point2");
  EXPECT(actual.namespaces().empty());
  EXPECT(actual.name()=="Point2");
  EXPECT(actual.category==Qualified::CLASS);
}

//******************************************************************************
TEST( Type, Constructor2 ) {
  Qualified actual("Point3",Qualified::CLASS);
  EXPECT(actual.namespaces().empty());
  EXPECT(actual.name()=="Point3");
  EXPECT(actual.category==Qualified::CLASS);
}

//******************************************************************************
TEST( Type, grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in actual
  Qualified actual;
  TypeGrammar type_g(actual);

  // a class type with 2 namespaces
  EXPECT(parse("gtsam::internal::Point2", type_g, space_p).full);
  EXPECT(actual==Qualified("gtsam","internal","Point2",Qualified::CLASS));
  actual.clear();

  // a class type with 1 namespace
  EXPECT(parse("gtsam::Point2", type_g, space_p).full);
  EXPECT(actual==Qualified("gtsam","Point2",Qualified::CLASS));
  actual.clear();

  // a class type with no namespaces
  EXPECT(parse("Point2", type_g, space_p).full);
  EXPECT(actual==Qualified("Point2",Qualified::CLASS));
  actual.clear();

  // a class type with no namespaces
  EXPECT(parse("VectorNotEigen", type_g, space_p).full);
  EXPECT(actual==Qualified("VectorNotEigen",Qualified::CLASS));
  actual.clear();

  // an Eigen type
  EXPECT(parse("Vector", type_g, space_p).full);
  EXPECT(actual==Qualified("Vector",Qualified::EIGEN));
  actual.clear();

  // a basic type
  EXPECT(parse("double", type_g, space_p).full);
  EXPECT(actual==Qualified("double",Qualified::BASIS));
  actual.clear();

  // void
  EXPECT(parse("void", type_g, space_p).full);
  EXPECT(actual==Qualified("void",Qualified::VOID));
  actual.clear();
}

//******************************************************************************
TEST( TypeList, grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in actual
  vector<Qualified> actual;
  TypeListGrammar<'{','}'> g(actual);

  EXPECT(parse("{gtsam::Point2}", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();

  EXPECT(parse("{}", g, space_p).full);
  EXPECT_LONGS_EQUAL(0, actual.size());
  actual.clear();

  EXPECT(parse("{char}", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();

  EXPECT(parse("{unsigned char}", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();

  EXPECT(parse("{Vector, Matrix}", g, space_p).full);
  EXPECT_LONGS_EQUAL(2, actual.size());
  EXPECT(actual[0]==Qualified("Vector",Qualified::EIGEN));
  EXPECT(actual[1]==Qualified("Matrix",Qualified::EIGEN));
  actual.clear();

  EXPECT(parse("{Point2}", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();

  EXPECT(parse("{Point2, Point3}", g, space_p).full);
  EXPECT_LONGS_EQUAL(2, actual.size());
  EXPECT(actual[0]==Qualified("Point2"));
  EXPECT(actual[1]==Qualified("Point3"));
  actual.clear();

  EXPECT(parse("{gtsam::Point2}", g, space_p).full);
  EXPECT_LONGS_EQUAL(1, actual.size());
  actual.clear();
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
