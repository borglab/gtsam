/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Template.h
 * @brief Template name
 * @author Frank Dellaert
 * @date Nov 11, 2014
 **/

#pragma once

#include <wrap/Qualified.h>

namespace wrap {

/// The template specification that goes before a method or a class
struct Template {
  std::string argName;
  std::vector<Qualified> argValues;
  void clear() {
    argName.clear();
    argValues.clear();
  }
};

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct TemplateGrammar: public classic::grammar<TemplateGrammar> {

  Template& result_; ///< successful parse will be placed in here

  TypeListGrammar<'{', '}'> argValues_g;

  /// Construct type grammar and specify where result is placed
  TemplateGrammar(Template& result) :
      result_(result), argValues_g(result.argValues) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: basic_rules<ScannerT> {

    typedef classic::rule<ScannerT> Rule;

    Rule templateArgValues_p;

    definition(TemplateGrammar const& self) {
      using namespace classic;
      templateArgValues_p = (str_p("template") >> '<'
          >> (basic_rules<ScannerT>::name_p)[assign_a(self.result_.argName)]
          >> '=' >> self.argValues_g >> '>');
    }

    Rule const& start() const {
      return templateArgValues_p;
    }

  };
};
// TemplateGrammar

}// \namespace wrap

