/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testReturnValue.cpp
 * @brief Unit test for ReturnValue class & parser
 * @author Frank Dellaert
 * @date Nov 30, 2014
 **/

#include <wrap/ReturnValue.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace wrap;

//******************************************************************************
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct ReturnValueGrammar: public classic::grammar<ReturnValueGrammar> {

  wrap::ReturnValue& result_; ///< successful parse will be placed in here

  TypeGrammar returnType1_g, returnType2_g;

  /// Construct type grammar and specify where result is placed
  ReturnValueGrammar(wrap::ReturnValue& result) :
      result_(result), returnType1_g(result.type1), returnType2_g(result.type2) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: basic_rules<ScannerT> {

    typedef classic::rule<ScannerT> Rule;

    Rule pair_p, returnValue_p;

    definition(ReturnValueGrammar const& self) {

      using namespace wrap;
      using namespace classic;

      pair_p = (str_p("pair") >> '<' >> self.returnType1_g >> ','
          >> self.returnType2_g >> '>')[assign_a(self.result_.isPair, T)];

      returnValue_p = pair_p | self.returnType1_g;
    }

    Rule const& start() const {
      return returnValue_p;
    }

  };
};
// ReturnValueGrammar

//******************************************************************************
TEST( ReturnValue, grammar ) {

  using classic::space_p;

  // Create type grammar that will place result in actual
  ReturnValue actual;
  ReturnValueGrammar g(actual);

  EXPECT(parse("VectorNotEigen", g, space_p).full);
  EXPECT(actual==ReturnValue(ReturnType("VectorNotEigen",Qualified::CLASS)));
  actual.clear();

  EXPECT(parse("double", g, space_p).full);
  EXPECT(actual==ReturnValue(ReturnType("double",Qualified::BASIS)));
  actual.clear();
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
