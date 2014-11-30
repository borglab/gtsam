/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMethod.cpp
 * @brief Unit test for Method class
 * @author Frank Dellaert
 * @date Nov 12, 2014
 **/

#include <wrap/Method.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace wrap;

//******************************************************************************
// Constructor
TEST( Method, Constructor ) {
  Method method;
}

//******************************************************************************
// addOverload
TEST( Method, addOverload ) {
  Method method;
  ArgumentList args;
  bool is_const = true;
  const ReturnValue retVal1(ReturnType("return_type1"));
  method.addOverload("myName", args, retVal1, is_const);
  const ReturnValue retVal2(ReturnType("return_type2"));
  method.addOverload("myName", args, retVal2, is_const);
  EXPECT_LONGS_EQUAL(2, method.nrOverloads());
}

//// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
//struct method_grammar: public classic::grammar<method_grammar> {
//
//  wrap::Method& result_; ///< successful parse will be placed in here
//
//  ArgumentList args;
//  Argument arg0, arg;
//  TypeGrammar argument_type_g;
//
//  ReturnType retType0, retType;
//  TypeGrammar returntype_g;
//
//  ReturnValue retVal0, retVal;
//
//  /// Construct type grammar and specify where result is placed
//  method_grammar(wrap::Method& result) :
//      result_(result), argument_type_g(arg.type), returntype_g(retType) {
//  }
//
//  /// Definition of type grammar
//  template<typename ScannerT>
//  struct definition: basic_rules<ScannerT> {
//
//    typedef classic::rule<ScannerT> Rule;
//
//    Rule templateArgValue_p, templateArgValues_p, argument_p, argumentList_p,
//        returnType1_p, returnType2_p, pair_p, returnValue_p, methodName_p,
//        method_p;
//
//    definition(method_grammar const& self) {
//
//      using namespace wrap;
//      using namespace classic;
//
////      Rule templateArgValue_p = type_grammar(self.templateArgValue);
////
////      // template<CALIBRATION = {gtsam::Cal3DS2}>
////      Rule templateArgValues_p = (str_p("template") >> '<' >> name_p >> '='
////          >> '{' >> !(templateArgValue_p >> *(',' >> templateArgValue_p)) >> '}'
////          >> '>');
////
//      // Create type grammar that will place result in actual
//      ArgumentList actual;
//      ArgumentListGrammar g(actual);
//
//      EXPECT(parse("(const gtsam::Point2& p4)", g, space_p).full);
//      EXPECT_LONGS_EQUAL(1, actual.size());
//      actual.clear();
//
//      returnType1_p = self.returntype_g //
//          [assign_a(self.retVal.type1, retType)] //
//          [assign_a(self.retType, self.retType0)];
//
//      returnType2_p = self.returntype_g //
//          [assign_a(self.retVal.type2, retType)] //
//          [assign_a(self.retType, self.retType0)];
//
//      pair_p = (str_p("pair") >> '<' >> returnType1_p >> ',' >> returnType2_p
//          >> '>')[assign_a(self.retVal.isPair, true)];
//
//      returnValue_p = pair_p | returnType1_p;
//
//      methodName_p = lexeme_d[(upper_p | lower_p) >> *(alnum_p | '_')];
//
//      // gtsam::Values retract(const gtsam::VectorValues& delta) const;
//      method_p =
////          !templateArgValues_p >>
//          (returnValue_p >> methodName_p >> '(' >> argumentList_p >> ')'
//              >> !str_p("const") >> ';' >> *basic_rules<ScannerT>::comments_p);
//    }
//
//    Rule const& start() const {
//      return method_p;
//    }
//
//  };
//};
//// method_grammar
//
////******************************************************************************
//TEST( Method, grammar ) {
//
//  using classic::space_p;
//
//  // Create type grammar that will place result in actual
//  Method actual;
//  method_grammar method_g(actual);
//
//  // a class type with namespaces
//  EXPECT(parse("double x() const;", method_g, space_p).full);
//}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
