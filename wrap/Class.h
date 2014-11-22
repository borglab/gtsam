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

#include "Constructor.h"
#include "Deconstructor.h"
#include "Method.h"
#include "StaticMethod.h"
#include "TypeAttributesTable.h"

#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>

#include <string>
#include <map>

namespace wrap {

/// Class has name, constructors, methods
class Class: public Qualified {

  typedef std::map<std::string, Method> Methods;
  Methods methods; ///< Class methods

public:

  typedef const std::string& Str;
  typedef std::map<std::string, StaticMethod> StaticMethods;

  // Then the instance variables are set directly by the Module constructor
  std::vector<std::string> templateArgs; ///< Template arguments
  std::string typedefName; ///< The name to typedef *from*, if this class is actually a typedef, i.e. typedef [typedefName] [name]
  bool isVirtual; ///< Whether the class is part of a virtual inheritance chain
  bool isSerializable; ///< Whether we can use boost.serialization to serialize the class - creates exports
  bool hasSerialization; ///< Whether we should create the serialization functions
  Qualified qualifiedParent; ///< The *single* parent
  StaticMethods static_methods; ///< Static methods
  Constructor constructor; ///< Class constructors
  Deconstructor deconstructor; ///< Deconstructor to deallocate C++ object
  bool verbose_; ///< verbose flag

  /// Constructor creates an empty class
  Class(bool verbose = true) :
      isVirtual(false), isSerializable(false), hasSerialization(false), deconstructor(
          verbose), verbose_(verbose) {
  }

  size_t nrMethods() const {
    return methods.size();
  }
  Method& method(Str name) {
    return methods.at(name);
  }
  bool exists(Str name) const {
    return methods.find(name) != methods.end();
  }

  // And finally MATLAB code is emitted, methods below called by Module::matlab_code
  void matlab_proxy(Str toolboxPath, Str wrapperName,
      const TypeAttributesTable& typeAttributes, FileWriter& wrapperFile,
      std::vector<std::string>& functionNames) const; ///< emit proxy class

  Class expandTemplate(const TemplateSubstitution& ts) const;

  std::vector<Class> expandTemplate(Str templateArg,
      const std::vector<Qualified>& instantiations) const;

  /// Add potentially overloaded, potentially templated method
  void addMethod(bool verbose, bool is_const, Str methodName,
      const ArgumentList& argumentList, const ReturnValue& returnValue,
      Str templateArgName, const std::vector<Qualified>& templateArgValues);

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
    os << "class " << cls.name << "{\n";
    os << cls.constructor << ";\n";
    BOOST_FOREACH(const StaticMethod& m, cls.static_methods | boost::adaptors::map_values)
      os << m << ";\n";
    BOOST_FOREACH(const Method& m, cls.methods | boost::adaptors::map_values)
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

} // \namespace wrap

