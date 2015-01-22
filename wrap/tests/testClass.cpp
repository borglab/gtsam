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
  args.push_back(Argument(Qualified("Vector", Qualified::EIGEN), "v"));
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

//******************************************************************************
TEST( Class, Grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in cls
  Class cls;
  Template t;
  ClassGrammar g(cls, t);

  EXPECT(parse("class Point2 {\n};", g, space_p).full);

  string markup(
      string("class Point2 {                \n")
          + string(" double x() const;            \n") // Method 1
          + string(" Matrix returnMatrix() const;   \n") // Method 2
          + string(" Point2 returnPoint2() const; \n") // Method 3
          + string(" static Vector returnVector(); \n") // Static Method 1
          + string("};"));

  EXPECT(parse(markup.c_str(), g, space_p).full);

  // check return types
  EXPECT(assert_equal("Point2", cls.name()));
  EXPECT(!cls.isVirtual);
  EXPECT(cls.namespaces().empty());
  LONGS_EQUAL(3, cls.nrMethods());
  LONGS_EQUAL(1, cls.static_methods.size());

  // Method 1
  Method m1 = cls.method("x");
  EXPECT(assert_equal("x", m1.name()));
  EXPECT(m1.isConst());
  LONGS_EQUAL(1, m1.nrOverloads());

  ReturnValue rv1 = m1.returnValue(0);
  EXPECT(!rv1.isPair);
  EXPECT(!rv1.type1.isPtr);
  EXPECT(assert_equal("double", rv1.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::BASIS, rv1.type1.category);

  // Method 2
  Method m2 = cls.method("returnMatrix");
  EXPECT(assert_equal("returnMatrix", m2.name()));
  EXPECT(m2.isConst());
  LONGS_EQUAL(1, m2.nrOverloads());

  ReturnValue rv2 = m2.returnValue(0);
  EXPECT(!rv2.isPair);
  EXPECT(!rv2.type1.isPtr);
  EXPECT(assert_equal("Matrix", rv2.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::EIGEN, rv2.type1.category);

  // Method 3
  Method m3 = cls.method("returnPoint2");
  EXPECT(assert_equal("returnPoint2", m3.name()));
  EXPECT(m3.isConst());
  LONGS_EQUAL(1, m3.nrOverloads());

  ReturnValue rv3 = m3.returnValue(0);
  EXPECT(!rv3.isPair);
  EXPECT(!rv3.type1.isPtr);
  EXPECT(assert_equal("Point2", rv3.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::CLASS, rv3.type1.category);

  // Static Method 1
  // static Vector returnVector();
  StaticMethod sm1 = cls.static_methods.at("returnVector");
  EXPECT(assert_equal("returnVector", sm1.name()));
  LONGS_EQUAL(1, sm1.nrOverloads());

  ReturnValue rv4 = sm1.returnValue(0);
  EXPECT(!rv4.isPair);
  EXPECT(!rv4.type1.isPtr);
  EXPECT(assert_equal("Vector", rv4.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::EIGEN, rv4.type1.category);
}

//******************************************************************************
TEST( Class, TemplateSubstition ) {

  using classic::space_p;

  // Create type grammar that will place result in cls
  Class cls;
  Template t;
  ClassGrammar g(cls, t);

  string markup(string("template<T = {void, double, Matrix, Point3}>"
      "class Point2 {"
      " T myMethod(const T& t) const;"
      "};"));

  EXPECT(parse(markup.c_str(), g, space_p).full);

  // Method 2
  Method m2 = cls.method("myMethod");
  EXPECT(assert_equal("myMethod", m2.name()));
  EXPECT(m2.isConst());
  LONGS_EQUAL(1, m2.nrOverloads());

  ReturnValue rv2 = m2.returnValue(0);
  EXPECT(!rv2.isPair);
  EXPECT(!rv2.type1.isPtr);
  EXPECT(assert_equal("T", rv2.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::CLASS, rv2.type1.category);

  // Generate some expected values for qualified types
  Qualified q_void("void", Qualified::VOID);
  Qualified q_double("double", Qualified::BASIS);
  Qualified q_Matrix("Matrix", Qualified::EIGEN);
  Qualified q_Point3("Point3", Qualified::CLASS);

  EXPECT_LONGS_EQUAL(4, t.nrValues());
  EXPECT(t.argName()=="T");
  EXPECT(t[0]==q_void);
  EXPECT(t[1]==q_double);
  EXPECT(t[2]==q_Matrix);
  EXPECT(t[3]==q_Point3);

  vector<Class> classes = cls.expandTemplate(t.argName(), t.argValues());

  // check the number of new classes is four
  EXPECT_LONGS_EQUAL(4, classes.size());

  // check return types 
  EXPECT( classes[0].method("myMethod").returnValue(0).type1 == q_void);
  EXPECT( classes[1].method("myMethod").returnValue(0).type1 == q_double);
  EXPECT( classes[2].method("myMethod").returnValue(0).type1 == q_Matrix);
  EXPECT( classes[3].method("myMethod").returnValue(0).type1 == q_Point3);

  // check the argument types
  EXPECT( classes[0].method("myMethod").argumentList(0)[0].type == q_void);
  EXPECT( classes[1].method("myMethod").argumentList(0)[0].type == q_double);
  EXPECT( classes[2].method("myMethod").argumentList(0)[0].type == q_Matrix);
  EXPECT( classes[3].method("myMethod").argumentList(0)[0].type == q_Point3);

}

//******************************************************************************
TEST( Class, TemplateSubstitionIntList ) {

  using classic::space_p;

  // Create type grammar that will place result in cls
  Class cls;
  Template t;
  ClassGrammar g(cls, t);

  string markup(string("template<N = {3,4}>"
      "class Point2 {"
      " void myMethod(Matrix A) const;"
      "};"));

  EXPECT(parse(markup.c_str(), g, space_p).full);

  vector<Class> classes = cls.expandTemplate(t.argName(), t.intList());

  // check the number of new classes is 2
  EXPECT_LONGS_EQUAL(2, classes.size());
}

//******************************************************************************
TEST(Class, Template) {

  using classic::space_p;

  // Create type grammar that will place result in cls
  Class cls;
  Template t;
  ClassGrammar g(cls, t);

  string markup(
      string(
          "template<T = {Vector, Matrix}>"
              " virtual class PriorFactor : gtsam::NoiseModelFactor {"
              "   PriorFactor(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel); "
              "   T prior() const; "
              "  void serialize() const; "
              "};"));

  EXPECT(parse(markup.c_str(), g, space_p).full);

}

//******************************************************************************
TEST( Class, Virtualness ) {
  using classic::space_p;
  Class cls;
  Template t;
  ClassGrammar g(cls, t);
  string markup("virtual class Point3 {};");
  EXPECT(parse(markup.c_str(), g, space_p).full);
  EXPECT(cls.isVirtual);
}
//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
