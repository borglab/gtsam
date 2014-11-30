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

static const bool T = true;

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct ArgumentGrammar: public classic::grammar<ArgumentGrammar> {

  wrap::Argument& result_; ///< successful parse will be placed in here
  TypeGrammar argument_type_g;

  /// Construct type grammar and specify where result is placed
  ArgumentGrammar(wrap::Argument& result) :
      result_(result), argument_type_g(result.type) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: basic_rules<ScannerT> {

    typedef classic::rule<ScannerT> Rule;

    Rule argument_p, argumentList_p;

    definition(ArgumentGrammar const& self) {

      using namespace wrap;
      using namespace classic;

      // NOTE: allows for pointers to all types
      // Slightly more permissive than before on basis/eigen type qualification
      // Also, currently parses Point2*&, can't make it work otherwise :-(
      argument_p = !str_p("const")[assign_a(self.result_.is_const, T)] //
          >> self.argument_type_g //
          >> !ch_p('*')[assign_a(self.result_.is_ptr, T)]
          >> !ch_p('&')[assign_a(self.result_.is_ref, T)]
          >> basic_rules<ScannerT>::name_p[assign_a(self.result_.name)];
    }

    Rule const& start() const {
      return argument_p;
    }

  };
};
// ArgumentGrammar

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

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct ArgumentListGrammar: public classic::grammar<ArgumentListGrammar> {

  wrap::ArgumentList& result_; ///< successful parse will be placed in here

  Argument arg0, arg;
  ArgumentGrammar argument_g;

  /// Construct type grammar and specify where result is placed
  ArgumentListGrammar(wrap::ArgumentList& result) :
      result_(result), argument_g(arg) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: basic_rules<ScannerT> {

    typedef classic::rule<ScannerT> Rule;

    Rule argument_p, argumentList_p;

    definition(ArgumentListGrammar const& self) {

      using namespace wrap;
      using namespace classic;

      // NOTE: allows for pointers to all types
      // Slightly more permissive than before on basis/eigen type qualification
      argument_p = self.argument_g //
          [push_back_a(self.result_, self.arg)] //
//          [assign_a(self.arg, self.arg0)]
           ;

      argumentList_p = '(' >> !argument_p >> *(',' >> argument_p) >> ')';
    }

    Rule const& start() const {
      return argumentList_p;
    }

  };
};
// ArgumentListGrammar

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

  EXPECT(parse("(unsigned char a)", g, space_p).full);

  EXPECT(parse("(Vector v, Matrix m)", g, space_p).full);

  EXPECT(parse("(Point2 p)", g, space_p).full);

  EXPECT(parse("(Point2 p1, Point3 p2)", g, space_p).full);

  EXPECT(parse("(gtsam::Point2 p3)", g, space_p).full);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
