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
class Template {
  std::string argName_;
  std::vector<Qualified> argValues_;
  std::vector<int> intList_;
  friend struct TemplateGrammar;
public:
  /// The only way to get values into a Template is via our friendly Grammar
  Template() {
  }
  void clear() {
    argName_.clear();
    argValues_.clear();
    intList_.clear();
  }
  const std::string& argName() const {
    return argName_;
  }
  const std::vector<int>& intList() const {
    return intList_;
  }
  const std::vector<Qualified>& argValues() const {
    return argValues_;
  }
  bool empty() const {
    return argValues_.empty() && intList_.empty();
  }
  size_t nrValues() const {
    return argValues_.size();
  }
  const Qualified& operator[](size_t i) const {
    return argValues_[i];
  }
  bool valid() const {
    return !argName_.empty() && argValues_.size() > 0;
  }

};

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct IntListGrammar: public classic::grammar<IntListGrammar > {

  typedef std::vector<int> IntList;
  IntList& result_; ///< successful parse will be placed in here

  /// Construct type grammar and specify where result is placed
  IntListGrammar(IntList& result) :
      result_(result) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition {

    classic::rule<ScannerT> integer_p, intList_p;

    definition(IntListGrammar const& self) {
      using namespace classic;

      integer_p = int_p[push_back_a(self.result_)];

      intList_p = '{' >> !integer_p >> *(',' >> integer_p) >> '}';
    }

    classic::rule<ScannerT> const& start() const {
      return intList_p;
    }

  };
};
// IntListGrammar

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct TemplateGrammar: public classic::grammar<TemplateGrammar> {

  Template& result_; ///< successful parse will be placed in here
  TypeListGrammar<'{', '}'> argValues_g; ///< TypeList parser
  IntListGrammar intList_g; ///< TypeList parser

  /// Construct type grammar and specify where result is placed
  TemplateGrammar(Template& result) :
      result_(result), argValues_g(result.argValues_), //
      intList_g(result.intList_) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: BasicRules<ScannerT> {

    classic::rule<ScannerT> templateArgValues_p;

    definition(TemplateGrammar const& self) {
      using classic::str_p;
      using classic::assign_a;
      templateArgValues_p = (str_p("template") >> '<'
          >> (BasicRules<ScannerT>::name_p)[assign_a(self.result_.argName_)]
          >> '=' >> (self.argValues_g | self.intList_g) >> '>');
    }

    classic::rule<ScannerT> const& start() const {
      return templateArgValues_p;
    }

  };
};
// TemplateGrammar

/// Cool initializer for tests
static inline boost::optional<Template> CreateTemplate(const std::string& s) {
  Template result;
  TemplateGrammar g(result);
  bool success = parse(s.c_str(), g, classic::space_p).full;
  if (success)
    return result;
  else
    return boost::none;
}

} // \namespace wrap

