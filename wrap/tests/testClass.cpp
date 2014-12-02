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

/* ************************************************************************* */
#include "../Module.h"
#include "../FileWriter.h"
#include "../TypeAttributesTable.h"
#include "../utilities.h"

//#define BOOST_SPIRIT_DEBUG
#include "../spirit.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include <boost/lambda/construct.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <algorithm>

using namespace std;
using namespace wrap;
using namespace BOOST_SPIRIT_CLASSIC_NS;
namespace bl = boost::lambda;
namespace fs = boost::filesystem;

// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct ClassGrammar: public classic::grammar<ClassGrammar> {

  Class& result_; ///< successful parse will be placed in here

  /// Construct type grammar and specify where result is placed
  ClassGrammar(Class& result) :
      result_(result) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: BasicRules<ScannerT> {

    using BasicRules<ScannerT>::name_p;
    using BasicRules<ScannerT>::className_p;
    using BasicRules<ScannerT>::comments_p;

    // NOTE: allows for pointers to all types
    ArgumentList args;
    ArgumentListGrammar argumentList_g;

    Constructor constructor0, constructor;

    ReturnValue retVal0, retVal;
    ReturnValueGrammar returnValue_g;

    // template<CALIBRATION = {gtsam::Cal3DS2}>
    Template methodTemplate, classTemplate;
    TemplateGrammar methodTemplate_g, classTemplate_g;

    string methodName;
    bool isConst, T, F;

    // Parent class
    Qualified possibleParent;
    TypeGrammar classParent_p;

    classic::rule<ScannerT> templateList_p, constructor_p, methodName_p,
        method_p, staticMethodName_p, static_method_p, functions_p, class_p;

    definition(ClassGrammar const& self) :
        argumentList_g(args), returnValue_g(retVal), //
        methodTemplate_g(methodTemplate), classTemplate_g(classTemplate), //
        T(true), F(false), classParent_p(possibleParent) {

      using namespace classic;
      bool verbose = false; // TODO

      // template<POSE, POINT>
      templateList_p = (str_p("template") >> '<'
          >> name_p[push_back_a(self.result_.templateArgs)]
          >> *(',' >> name_p[push_back_a(self.result_.templateArgs)]) >> '>');

      // parse class constructor
      constructor_p =
          (className_p >> argumentList_g >> ';' >> !comments_p)[bl::bind(
              &Constructor::push_back, bl::var(constructor), bl::var(args))][clear_a(
              args)];

      // TODO why is this not used anywhere?
//      vector<string> namespaces_return; /// namespace for current return type
//      Rule namespace_ret_p = namespace_p[push_back_a(namespaces_return)]
//          >> str_p("::");

      methodName_p = lexeme_d[(upper_p | lower_p) >> *(alnum_p | '_')];

      // gtsam::Values retract(const gtsam::VectorValues& delta) const;
      method_p = !methodTemplate_g
          >> (returnValue_g >> methodName_p[assign_a(methodName)]
              >> argumentList_g >> !str_p("const")[assign_a(isConst, T)] >> ';'
              >> *comments_p) //
          [bl::bind(&Class::addMethod, bl::var(self.result_), verbose,
              bl::var(isConst), bl::var(methodName), bl::var(args),
              bl::var(retVal), bl::var(methodTemplate))] //
          [assign_a(retVal, retVal0)] //
          [clear_a(args)][clear_a(methodTemplate)] //
          [assign_a(isConst, F)];

      staticMethodName_p = lexeme_d[(upper_p | lower_p) >> *(alnum_p | '_')];

      static_method_p = (str_p("static") >> returnValue_g
          >> staticMethodName_p[assign_a(methodName)] >> argumentList_g >> ';'
          >> *comments_p) //
          [bl::bind(&StaticMethod::addOverload,
              bl::var(self.result_.static_methods)[bl::var(methodName)],
              bl::var(methodName), bl::var(args), bl::var(retVal), boost::none,
              verbose)] //
          [assign_a(retVal, retVal0)][clear_a(args)];

      functions_p = constructor_p | method_p | static_method_p;

      // parse a full class
      class_p = (!(classTemplate_g[push_back_a(self.result_.templateArgs,
          classTemplate.argName())] | templateList_p)
          >> !(str_p("virtual")[assign_a(self.result_.isVirtual, true)])
          >> str_p("class") >> className_p[assign_a(self.result_.name_)]
          >> ((':' >> classParent_p >> '{')[bl::bind(&Class::assignParent,
              bl::var(self.result_), bl::var(possibleParent))][clear_a(
              possibleParent)] | '{') >> *(functions_p | comments_p)
          >> str_p("};")) //
          [bl::bind(&Constructor::initializeOrCheck, bl::var(constructor),
              bl::var(self.result_.name_), boost::none, verbose)][assign_a(
              self.result_.constructor, constructor)] //
          [assign_a(self.result_.deconstructor.name, self.result_.name_)] //
          [clear_a(classTemplate)] //
          [assign_a(constructor, constructor0)];
    }

    classic::rule<ScannerT> const& start() const {
      return class_p;
    }

  };
};
// ClassGrammar

//******************************************************************************
TEST( Class, Grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in cls
  Class cls;
  ClassGrammar g(cls);

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

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
