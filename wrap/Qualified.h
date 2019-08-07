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
#include <iostream>

namespace wrap {

/**
 * Class to encapuslate a qualified name, i.e., with (nested) namespaces
 */
class Qualified {

//protected:
public:

  std::vector<std::string> namespaces_; ///< Stack of namespaces
  std::string name_; ///< type name
  static std::vector<Qualified> BasicTypedefs;

  friend struct TypeGrammar;
  friend class TemplateSubstitution;

public:

  /// the different categories
  typedef enum {
    CLASS = 1, EIGEN = 2, BASIS = 3, VOID = 4
  } Category;
  Category category;

  /// Default constructor
  Qualified() :
      category(VOID) {
  }

  /// Construct from name and optional category
  Qualified(const std::string& n, Category c = CLASS) :
      name_(n), category(c) {
  }

  /// Construct from scoped name and optional category
  Qualified(const std::string& ns1, const std::string& n, Category c = CLASS) :
      name_(n), category(c) {
    namespaces_.push_back(ns1);
  }

  /// Construct from doubly scoped name and optional category
  Qualified(const std::string& ns1, const std::string& ns2,
      const std::string& n, Category c = CLASS) :
      name_(n), category(c) {
    namespaces_.push_back(ns1);
    namespaces_.push_back(ns2);
  }

  /// Construct from arbitrarily scoped name
  Qualified(std::vector<std::string> ns, const std::string& name) :
      namespaces_(ns), name_(name), category(CLASS) {
  }

  // Destructor
  virtual ~Qualified() {}

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

  bool match(const std::vector<std::string>& templateArgs) const {
    for(const std::string& s: templateArgs)
      if (match(s)) return true;
    return false;
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

  bool isScalar() const {
    return (name() == "bool" || name() == "char"
        || name() == "unsigned char" || name() == "int"
        || name() == "size_t" || name() == "double");
  }

  bool isVoid() const {
    return name() == "void";
  }

  bool isString() const {
    return name() == "string";
  }

  bool isEigen() const {
    return name() == "Vector" || name() == "Matrix";
  }

  bool isBasicTypedef() const {
    return std::find(Qualified::BasicTypedefs.begin(),
                     Qualified::BasicTypedefs.end(),
                     *this) != Qualified::BasicTypedefs.end();
  }

  bool isNonBasicType() const {
    return name() != "This" && !isString() && !isScalar() && !isEigen() &&
           !isVoid() && !isBasicTypedef();
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

  /// Return a qualified namespace using given delimiter
  std::string qualifiedNamespaces(const std::string& delimiter = "") const {
    std::string result;
    for (std::size_t i = 0; i < namespaces_.size(); ++i)
      result += (namespaces_[i] + ((i<namespaces_.size()-1)?delimiter:""));
    return result;
  }

  /// Return a qualified string using given delimiter
  std::string qualifiedName(const std::string& delimiter = "", size_t fromLevel = 0) const {
    std::string result;
    for (std::size_t i = fromLevel; i < namespaces_.size(); ++i)
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

  /// name of Cython classes in pxd
  /// Normal classes: innerNamespace_ClassName, e.g. GaussianFactor, noiseModel_Gaussian
  /// Eigen type: Vector --> VectorXd, Matrix --> MatrixXd
  std::string pxdClassName() const {
    if (isEigen())
      return name_ + "Xd";
    else if (isNonBasicType())
      return "C" + qualifiedName("_", 1);
    else return name_;
  }

  /// name of Python classes in pyx
  /// They have the same name with the corresponding Cython classes in pxd
  /// But note that they are different: These are Python classes in the pyx file
  /// To refer to a Cython class in pyx, we need to add "pxd.", e.g. pxd.noiseModel_Gaussian
  /// see the other function pxd_class_in_pyx for that purpose.
  std::string pyxClassName() const {
    if (isEigen())
      return name_;
    else
      return qualifiedName("_", 1);
  }

  /// Python type of function arguments in pyx to interface with normal python scripts
  /// Eigen types become np.ndarray (There's no Eigen types, e.g. VectorXd, in
  /// Python. We have to pass in numpy array in the arguments, which will then be
  /// converted to Eigen types in Cython)
  std::string pyxArgumentType() const {
    if (isEigen())
      return "np.ndarray";
    else
      return qualifiedName("_", 1);
  }

  /// return the Cython class in pxd corresponding to a Python class in pyx
  std::string pxd_class_in_pyx() const {
    if (isNonBasicType()) {
      return pxdClassName();
    } else if (isEigen()) {
      return name_ + "Xd";
    } else  // basic types and not Eigen
      return name_;
  }

  /// the internal Cython shared obj in a Python class wrappper
  std::string shared_pxd_obj_in_pyx() const {
    return pxdClassName() + "_";
  }

  std::string make_shared_pxd_class_in_pyx() const {
    return "make_shared[" + pxd_class_in_pyx() + "]";
  }

  std::string shared_pxd_class_in_pyx() const {
    return "shared_ptr[" + pxd_class_in_pyx() + "]";
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

