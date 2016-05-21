/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Class.h
 * @brief describe the C++ class that is being wrapped
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#pragma once

#include "spirit.h"
#include "Template.h"
#include "Constructor.h"
#include "Deconstructor.h"
#include "Method.h"
#include "StaticMethod.h"
#include "TypeAttributesTable.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace bl = boost::lambda;

#include <boost/range/adaptor/map.hpp>
#include <boost/optional.hpp>

#include <string>
#include <map>

namespace wrap {

/// Class has name, constructors, methods
class Class: public Qualified {

public:
  typedef const std::string& Str;
  typedef std::map<std::string, Method> Methods;
  typedef std::map<std::string, StaticMethod> StaticMethods;

private:

  boost::optional<Qualified> parentClass; ///< The *single* parent
  Methods methods_; ///< Class methods
  Method& mutableMethod(Str key);

public:

  StaticMethods static_methods; ///< Static methods

  // Then the instance variables are set directly by the Module constructor
  std::vector<std::string> templateArgs; ///< Template arguments
  std::string typedefName; ///< The name to typedef *from*, if this class is actually a typedef, i.e. typedef [typedefName] [name]
  bool isVirtual; ///< Whether the class is part of a virtual inheritance chain
  bool isSerializable; ///< Whether we can use boost.serialization to serialize the class - creates exports
  bool hasSerialization; ///< Whether we should create the serialization functions
  Constructor constructor; ///< Class constructors
  Deconstructor deconstructor; ///< Deconstructor to deallocate C++ object
  bool verbose_; ///< verbose flag

  /// Constructor creates an empty class
  Class(bool verbose = true) :
      parentClass(boost::none), isVirtual(false), isSerializable(false), hasSerialization(
          false), deconstructor(verbose), verbose_(verbose) {
  }

  void assignParent(const Qualified& parent);

  boost::optional<std::string> qualifiedParent() const;

  size_t nrMethods() const {
    return methods_.size();
  }

  const Method& method(Str key) const;

  bool exists(Str name) const {
    return methods_.find(name) != methods_.end();
  }

  // And finally MATLAB code is emitted, methods below called by Module::matlab_code
  void matlab_proxy(Str toolboxPath, Str wrapperName,
      const TypeAttributesTable& typeAttributes, FileWriter& wrapperFile,
      std::vector<std::string>& functionNames) const; ///< emit proxy class

  Class expandTemplate(const TemplateSubstitution& ts) const;

  std::vector<Class> expandTemplate(Str templateArg,
      const std::vector<Qualified>& instantiations) const;

  // Create new classes with integer template arguments
  std::vector<Class> expandTemplate(Str templateArg,
      const std::vector<int>& integers) const;

  /// Add potentially overloaded, potentially templated method
  void addMethod(bool verbose, bool is_const, Str methodName,
      const ArgumentList& argumentList, const ReturnValue& returnValue,
      const Template& tmplate);

  /// Post-process classes for serialization markers
  void erase_serialization(); // non-const !

  /// verify all of the function arguments
  void verifyAll(std::vector<std::string>& functionNames,
      bool& hasSerialiable) const;

  void appendInheritedMethods(const Class& cls,
      const std::vector<Class>& classes);

  /// The typedef line for this class, if this class is a typedef, otherwise returns an empty string.
  std::string getTypedef() const;

  /// Returns the string for an export flag
  std::string getSerializationExport() const;

  /// Creates a member function that performs serialization
  void serialization_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
      Str wrapperName, std::vector<std::string>& functionNames) const;

  /// Creates a static member function that performs deserialization
  void deserialization_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
      Str wrapperName, std::vector<std::string>& functionNames) const;

  // emit python wrapper
  void python_wrapper(FileWriter& wrapperFile) const;

  friend std::ostream& operator<<(std::ostream& os, const Class& cls) {
    os << "class " << cls.name() << "{\n";
    os << cls.constructor << ";\n";
    for(const StaticMethod& m: cls.static_methods | boost::adaptors::map_values)
      os << m << ";\n";
    for(const Method& m: cls.methods_ | boost::adaptors::map_values)
      os << m << ";\n";
    os << "};" << std::endl;
    return os;
  }

private:

  void pointer_constructor_fragments(FileWriter& proxyFile,
      FileWriter& wrapperFile, Str wrapperName,
      std::vector<std::string>& functionNames) const;

  void comment_fragment(FileWriter& proxyFile) const;
};

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct ClassGrammar: public classic::grammar<ClassGrammar> {

  Class& cls_; ///< successful parse will be placed in here
  Template& template_; ///< result needs to be visible outside

  /// Construct type grammar and specify where result is placed
  ClassGrammar(Class& cls, Template& t) :
      cls_(cls), template_(t) {
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

    Template methodTemplate;
    TemplateGrammar methodTemplate_g, classTemplate_g;

    std::string methodName;
    bool isConst, T, F;

    // Parent class
    Qualified possibleParent;
    TypeGrammar classParent_g;

    classic::rule<ScannerT> constructor_p, methodName_p, method_p,
        staticMethodName_p, static_method_p, templateList_p, classParent_p,
        functions_p, class_p;

    definition(ClassGrammar const& self) :
        argumentList_g(args), returnValue_g(retVal), //
        methodTemplate_g(methodTemplate), classTemplate_g(self.template_), //
        T(true), F(false), classParent_g(possibleParent) {

      using namespace classic;
      bool verbose = false; // TODO

      // ConstructorGrammar
      constructor_p = (className_p >> argumentList_g >> ';' >> !comments_p) //
          [bl::bind(&Constructor::push_back, bl::var(constructor),
              bl::var(args))] //
          [clear_a(args)];

      // MethodGrammar
      methodName_p = lexeme_d[(upper_p | lower_p) >> *(alnum_p | '_')];

      // gtsam::Values retract(const gtsam::VectorValues& delta) const;
      method_p = !methodTemplate_g
          >> (returnValue_g >> methodName_p[assign_a(methodName)]
              >> argumentList_g >> !str_p("const")[assign_a(isConst, T)] >> ';'
              >> *comments_p) //
          [bl::bind(&Class::addMethod, bl::var(self.cls_), verbose,
              bl::var(isConst), bl::var(methodName), bl::var(args),
              bl::var(retVal), bl::var(methodTemplate))] //
          [assign_a(retVal, retVal0)][clear_a(args)] //
          [clear_a(methodTemplate)][assign_a(isConst, F)];

      // StaticMethodGrammar
      staticMethodName_p = lexeme_d[(upper_p | lower_p) >> *(alnum_p | '_')];

      static_method_p = (str_p("static") >> returnValue_g
          >> staticMethodName_p[assign_a(methodName)] >> argumentList_g >> ';'
          >> *comments_p) //
          [bl::bind(&StaticMethod::addOverload,
              bl::var(self.cls_.static_methods)[bl::var(methodName)],
              bl::var(methodName), bl::var(args), bl::var(retVal), boost::none,
              verbose)] //
          [assign_a(retVal, retVal0)][clear_a(args)];

      // template<POSE, POINT>
      templateList_p = (str_p("template") >> '<'
          >> name_p[push_back_a(self.cls_.templateArgs)]
          >> *(',' >> name_p[push_back_a(self.cls_.templateArgs)]) >> '>');

      // parse a full class
      classParent_p = (':' >> classParent_g >> '{') //
          [bl::bind(&Class::assignParent, bl::var(self.cls_),
              bl::var(possibleParent))][clear_a(possibleParent)];

      functions_p = constructor_p | method_p | static_method_p;

      // parse a full class
      class_p = (!(classTemplate_g[push_back_a(self.cls_.templateArgs,
          self.template_.argName())] | templateList_p)
          >> !(str_p("virtual")[assign_a(self.cls_.isVirtual, T)])
          >> str_p("class") >> className_p[assign_a(self.cls_.name_)]
          >> (classParent_p | '{') >> //
          *(functions_p | comments_p) >> str_p("};")) //
          [bl::bind(&Constructor::initializeOrCheck, bl::var(constructor),
              bl::var(self.cls_.name_), boost::none, verbose)][assign_a(
              self.cls_.constructor, constructor)] //
          [assign_a(self.cls_.deconstructor.name, self.cls_.name_)] //
          [assign_a(constructor, constructor0)];
    }

    classic::rule<ScannerT> const& start() const {
      return class_p;
    }

  };
};
// ClassGrammar

}// \namespace wrap

