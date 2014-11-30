/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Qualified.h
 * @brief Qualified name
 * @author Frank Dellaert
 * @date Nov 11, 2014
 **/

#pragma once

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_push_back_actor.hpp>
#include <boost/spirit/include/classic_clear_actor.hpp>
#include <boost/spirit/include/classic_assign_actor.hpp>
#include <boost/spirit/include/classic_confix.hpp>

namespace classic = BOOST_SPIRIT_CLASSIC_NS;

#include <string>
#include <vector>

namespace wrap {

/**
 * Class to encapuslate a qualified name, i.e., with (nested) namespaces
 */
struct Qualified {

  std::vector<std::string> namespaces; ///< Stack of namespaces
  std::string name; ///< type name

  /// the different categories
  typedef enum {
    CLASS = 1, EIGEN = 2, BASIS = 3, VOID = 4
  } Category;
  Category category;

  Qualified() :
      category(VOID) {
  }

  Qualified(const std::string& name_, Category c = CLASS) :
      name(name_), category(c) {
  }

  bool operator==(const Qualified& other) const {
    return namespaces == other.namespaces && name == other.name
        && category == other.category;
  }

  bool empty() const {
    return namespaces.empty() && name.empty();
  }

  void clear() {
    namespaces.clear();
    name.clear();
  }

  bool operator!=(const Qualified& other) const {
    return other.name != name || other.namespaces != namespaces;
  }

  /// Return a qualified string using given delimiter
  std::string qualifiedName(const std::string& delimiter = "") const {
    std::string result;
    for (std::size_t i = 0; i < namespaces.size(); ++i)
      result += (namespaces[i] + delimiter);
    result += name;
    return result;
  }

  /// Return a matlab file name, i.e. "toolboxPath/+ns1/+ns2/name.m"
  std::string matlabName(const std::string& toolboxPath) const {
    std::string result = toolboxPath;
    for (std::size_t i = 0; i < namespaces.size(); ++i)
      result += ("/+" + namespaces[i]);
    result += "/" + name + ".m";
    return result;
  }

  friend std::ostream& operator<<(std::ostream& os, const Qualified& q) {
    os << q.qualifiedName("::");
    return os;
  }

};

template<typename ScannerT>
struct basic_rules {

  typedef classic::rule<ScannerT> Rule;

  Rule comments_p, basisType_p, eigenType_p, keywords_p, stlType_p, name_p,
      className_p, namepsace_p;

  basic_rules() {

    using namespace classic;

    comments_p = comment_p("/*", "*/") | comment_p("//", eol_p);

    basisType_p = (str_p("string") | "bool" | "size_t" | "int" | "double"
        | "char" | "unsigned char");

    eigenType_p = (str_p("Vector") | "Matrix");

    keywords_p =
        (str_p("const") | "static" | "namespace" | "void" | basisType_p);

    stlType_p = (str_p("vector") | "list");

    name_p = lexeme_d[alpha_p >> *(alnum_p | '_')];

    className_p = (lexeme_d[upper_p >> *(alnum_p | '_')] - eigenType_p
        - keywords_p) | stlType_p;

    namepsace_p = lexeme_d[lower_p >> *(alnum_p | '_')] - keywords_p;
  }
};

// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct type_grammar: public classic::grammar<type_grammar> {

  wrap::Qualified& result_; ///< successful parse will be placed in here

  /// Construct type grammar and specify where result is placed
  type_grammar(wrap::Qualified& result) :
      result_(result) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: basic_rules<ScannerT> {

    typedef classic::rule<ScannerT> Rule;

    Rule void_p, my_basisType_p, my_eigenType_p, namespace_del_p, class_p,
        type_p;

    definition(type_grammar const& self) {

      using namespace wrap;
      using namespace classic;

      // HACK: use const values instead of using enums themselves - somehow this doesn't result in values getting assigned to gibberish
      static const Qualified::Category EIGEN = Qualified::EIGEN;
      static const Qualified::Category BASIS = Qualified::BASIS;
      static const Qualified::Category CLASS = Qualified::CLASS;
      static const Qualified::Category VOID = Qualified::VOID;

      void_p = str_p("void")[assign_a(self.result_.name)] //
          [assign_a(self.result_.category, VOID)];

      my_basisType_p = basic_rules<ScannerT>::basisType_p //
          [assign_a(self.result_.name)] //
          [assign_a(self.result_.category, BASIS)];

      my_eigenType_p = basic_rules<ScannerT>::eigenType_p //
          [assign_a(self.result_.name)] //
          [assign_a(self.result_.category, EIGEN)];

      namespace_del_p = basic_rules<ScannerT>::namepsace_p //
      [push_back_a(self.result_.namespaces)] >> str_p("::");

      class_p = *namespace_del_p >> basic_rules<ScannerT>::className_p //
          [assign_a(self.result_.name)] //
          [assign_a(self.result_.category, CLASS)];

      type_p = void_p | my_basisType_p | my_eigenType_p | class_p;
    }

    Rule const& start() const {
      return type_p;
    }

  };
};
// type_grammar

}// \namespace wrap

