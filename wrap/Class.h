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

#include <string>
#include <map>

#include "Constructor.h"
#include "Deconstructor.h"
#include "Method.h"
#include "StaticMethod.h"
#include "TypeAttributesTable.h"

namespace wrap {

/// Class has name, constructors, methods
struct Class : public Qualified {
  typedef std::map<std::string, Method> Methods;
  typedef std::map<std::string, StaticMethod> StaticMethods;

  /// Constructor creates an empty class
  Class(bool verbose=true) : isVirtual(false), isSerializable(false), hasSerialization(false), verbose_(verbose) {}

  // Then the instance variables are set directly by the Module constructor
  std::vector<std::string> templateArgs;    ///< Template arguments
  std::string typedefName;                  ///< The name to typedef *from*, if this class is actually a typedef, i.e. typedef [typedefName] [name]
  bool isVirtual;                           ///< Whether the class is part of a virtual inheritance chain
  bool isSerializable;                      ///< Whether we can use boost.serialization to serialize the class - creates exports
  bool hasSerialization;                    ///< Whether we should create the serialization functions
  Qualified qualifiedParent;                ///< The *single* parent
  Methods methods;                          ///< Class methods
  StaticMethods static_methods;             ///< Static methods
  Constructor constructor;                  ///< Class constructors
  Deconstructor deconstructor;              ///< Deconstructor to deallocate C++ object
  bool verbose_;                            ///< verbose flag

  // And finally MATLAB code is emitted, methods below called by Module::matlab_code
  void matlab_proxy(const std::string& toolboxPath, const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
    FileWriter& wrapperFile, std::vector<std::string>& functionNames) const;          ///< emit proxy class

  Class expandTemplate(const std::string& templateArg,
      const Qualified& instantiation,
      const Qualified& expandedClass) const;

  std::vector<Class> expandTemplate(const std::string& templateArg,
      const std::vector<Qualified >& instantiations) const;

  // The typedef line for this class, if this class is a typedef, otherwise returns an empty string.
  std::string getTypedef() const;

  // Returns the string for an export flag
  std::string getSerializationExport() const;

  // Creates a member function that performs serialization
  void serialization_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
      const std::string& wrapperName, std::vector<std::string>& functionNames) const;

  // Creates a static member function that performs deserialization
  void deserialization_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
      const std::string& wrapperName, std::vector<std::string>& functionNames) const;

private:
  void pointer_constructor_fragments(FileWriter& proxyFile, FileWriter& wrapperFile, const std::string& wrapperName, std::vector<std::string>& functionNames) const;
    void comment_fragment(FileWriter& proxyFile) const;
};

} // \namespace wrap

