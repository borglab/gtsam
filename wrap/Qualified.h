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

#include <string>
#include <vector>

namespace wrap {

/**
 * Class to encapuslate a qualified name, i.e., with (nested) namespaces
 */
struct Qualified {

  std::vector<std::string> namespaces; ///< Stack of namespaces
  std::string name; ///< type name

  Qualified(const std::string& name_ = "") :
      name(name_) {
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

} // \namespace wrap

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_push_back_actor.hpp>
#include <boost/spirit/include/classic_clear_actor.hpp>
#include <boost/spirit/include/classic_assign_actor.hpp>

namespace classic = BOOST_SPIRIT_CLASSIC_NS;

// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct type_grammar: public classic::grammar<type_grammar> {

  wrap::Qualified& result_; ///< successful parse will be placed in here

  /// Construct type grammar and specify where result is placed
  type_grammar(wrap::Qualified& result) :
      result_(result) {
  }

/// Definition of type grammar
  template<typename ScannerT>
  struct definition {

    typedef classic::rule<ScannerT> Rule;

    Rule void_p, basisType_p, eigenType_p, keywords_p, stlType_p, className_p,
        namepsace_p, namespace_del_p, class_p, type_p;

    definition(type_grammar const& self) {

      using namespace classic;

      void_p = str_p("void")[assign_a(self.result_.name)];

      basisType_p = (str_p("string") | "bool" | "size_t" | "int" | "double"
          | "char" | "unsigned char")[assign_a(self.result_.name)];

      eigenType_p = (str_p("Vector") | "Matrix")[assign_a(self.result_.name)];

      keywords_p = (str_p("const") | "static" | "namespace" | "void"
          | basisType_p);

      stlType_p = (str_p("vector") | "list");

      className_p = (lexeme_d[upper_p >> *(alnum_p | '_')] - eigenType_p
          - keywords_p) | stlType_p;

      namepsace_p = lexeme_d[lower_p >> *(alnum_p | '_')] - keywords_p;

      namespace_del_p = namepsace_p[push_back_a(self.result_.namespaces)]
          >> str_p("::");

      class_p = *namespace_del_p >> className_p[assign_a(self.result_.name)];

      type_p = eps_p[clear_a(self.result_)] //
      >> void_p | basisType_p | eigenType_p | class_p;
    }

    Rule const& start() const {
      return type_p;
    }

  };
};


