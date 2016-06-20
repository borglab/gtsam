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

#include <wrap/spirit.h>
#include <string>
#include <vector>

namespace wrap {

/**
 * Class to encapuslate a qualified name, i.e., with (nested) namespaces
 */
class Qualified {

//protected:
public:

  std::vector<std::string> namespaces_; ///< Stack of namespaces
  std::string name_; ///< type name

  friend struct TypeGrammar;
  friend class TemplateSubstitution;

public:

  /// the different categories
  typedef enum {
    CLASS = 1, EIGEN = 2, BASIS = 3, VOID = 4
  } Category;
  Category category;

  Qualified() :
      category(VOID) {
  }

  Qualified(const std::string& n, Category c = CLASS) :
      name_(n), category(c) {
  }

  Qualified(const std::string& ns1, const std::string& ns2,
      const std::string& n, Category c = CLASS) :
      name_(n), category(c) {
    namespaces_.push_back(ns1);
    namespaces_.push_back(ns2);
  }

  Qualified(const std::string& ns1, const std::string& n, Category c = CLASS) :
      name_(n), category(c) {
    namespaces_.push_back(ns1);
  }

  Qualified(std::vector<std::string> ns, const std::string& name) :
      namespaces_(ns), name_(name), category(CLASS) {
  }

  std::string name() const {
    return name_;
  }

  std::vector<std::string> namespaces() const {
    return namespaces_;
  }

  // Qualified is 'abused' as template argument name as well
  // this function checks whether *this matches with templateArg
  bool match(const std::string& templateArg) const {
    return (name_ == templateArg && namespaces_.empty()); //TODO && category == CLASS);
  }

  void rename(const Qualified& q) {
    namespaces_ = q.namespaces_;
    name_ = q.name_;
    category = q.category;
  }

  void expand(const std::string& expansion) {
    name_ += expansion;
  }

  bool operator==(const Qualified& other) const {
    return namespaces_ == other.namespaces_ && name_ == other.name_
        && category == other.category;
  }

  bool empty() const {
    return namespaces_.empty() && name_.empty();
  }

  virtual void clear() {
    namespaces_.clear();
    name_.clear();
    category = VOID;
  }

public:

  static Qualified MakeClass(std::vector<std::string> namespaces,
      const std::string& name) {
    return Qualified(namespaces, name);
  }

  static Qualified MakeEigen(const std::string& name) {
    return Qualified(name, EIGEN);
  }

  static Qualified MakeBasis(const std::string& name) {
    return Qualified(name, BASIS);
  }

  static Qualified MakeVoid() {
    return Qualified("void", VOID);
  }

  /// Return a qualified string using given delimiter
  std::string qualifiedName(const std::string& delimiter = "") const {
    std::string result;
    for (std::size_t i = 0; i < namespaces_.size(); ++i)
      result += (namespaces_[i] + delimiter);
    result += name_;
    return result;
  }

  /// Return a matlab file name, i.e. "toolboxPath/+ns1/+ns2/name.m"
  std::string matlabName(const std::string& toolboxPath) const {
    std::string result = toolboxPath;
    for (std::size_t i = 0; i < namespaces_.size(); ++i)
      result += ("/+" + namespaces_[i]);
    result += "/" + name_ + ".m";
    return result;
  }

  friend std::ostream& operator<<(std::ostream& os, const Qualified& q) {
    os << q.qualifiedName("::");
    return os;
  }

};

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct TypeGrammar: classic::grammar<TypeGrammar> {

  wrap::Qualified& result_; ///< successful parse will be placed in here

  /// Construct type grammar and specify where result is placed
  TypeGrammar(wrap::Qualified& result) :
      result_(result) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: BasicRules<ScannerT> {

    typedef classic::rule<ScannerT> Rule;

    Rule void_p, basisType_p, eigenType_p, namespace_del_p, class_p, type_p;

    definition(TypeGrammar const& self) {

      using namespace wrap;
      using namespace classic;
      typedef BasicRules<ScannerT> Basic;

      // HACK: use const values instead of using enums themselves - somehow this doesn't result in values getting assigned to gibberish
      static const Qualified::Category EIGEN = Qualified::EIGEN;
      static const Qualified::Category BASIS = Qualified::BASIS;
      static const Qualified::Category CLASS = Qualified::CLASS;
      static const Qualified::Category VOID = Qualified::VOID;

      void_p = str_p("void") //
          [assign_a(self.result_.name_)] //
          [assign_a(self.result_.category, VOID)];

      basisType_p = Basic::basisType_p //
          [assign_a(self.result_.name_)] //
          [assign_a(self.result_.category, BASIS)];

      eigenType_p = Basic::eigenType_p //
          [assign_a(self.result_.name_)] //
          [assign_a(self.result_.category, EIGEN)];

      namespace_del_p = Basic::namespace_p //
      [push_back_a(self.result_.namespaces_)] >> str_p("::");

      class_p = *namespace_del_p >> Basic::className_p //
          [assign_a(self.result_.name_)] //
          [assign_a(self.result_.category, CLASS)];

      type_p = void_p | basisType_p | class_p | eigenType_p;
    }

    Rule const& start() const {
      return type_p;
    }

  };
};
// type_grammar

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
template<char OPEN, char CLOSE>
struct TypeListGrammar: public classic::grammar<TypeListGrammar<OPEN, CLOSE> > {

  typedef std::vector<wrap::Qualified> TypeList;
  TypeList& result_; ///< successful parse will be placed in here

  /// Construct type grammar and specify where result is placed
  TypeListGrammar(TypeList& result) :
      result_(result) {
  }

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition {

    wrap::Qualified type; ///< temporary for use during parsing
    TypeGrammar type_g; ///< Individual Type grammars

    classic::rule<ScannerT> type_p, typeList_p;

    definition(TypeListGrammar const& self) :
        type_g(type) {
      using namespace classic;

      type_p = type_g[push_back_a(self.result_, type)][clear_a(type)];

      typeList_p = OPEN >> !type_p >> *(',' >> type_p) >> CLOSE;
    }

    classic::rule<ScannerT> const& start() const {
      return typeList_p;
    }

  };
};
// TypeListGrammar

/* ************************************************************************* */
// Needed for other parsers in Argument.h and ReturnType.h
static const bool T = true;

} // \namespace wrap

