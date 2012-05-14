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
 **/

#pragma once

#include <string>

#include "Constructor.h"
#include "Deconstructor.h"
#include "Method.h"
#include "StaticMethod.h"

namespace wrap {

/// Class has name, constructors, methods
struct Class {
  /// Constructor creates an empty class
  Class(bool verbose=true) : verbose_(verbose) {}

	// Then the instance variables are set directly by the Module constructor
  std::string name;                         ///< Class name
  std::vector<Constructor> constructors;    ///< Class constructors
  std::vector<Method> methods;              ///< Class methods
  std::vector<StaticMethod> static_methods; ///< Static methods
  std::vector<std::string> namespaces;      ///< Stack of namespaces
  std::vector<std::string> using_namespaces; ///< default namespaces
  std::vector<std::string> includes;        ///< header include overrides
  Deconstructor d;
  bool verbose_;                            ///< verbose flag

  // And finally MATLAB code is emitted, methods below called by Module::matlab_code
  void matlab_proxy(const std::string& classFile) const;          ///< emit proxy class
  void matlab_constructors(const std::string& toolboxPath) const;   ///< emit constructor wrappers
  void matlab_deconstructor(const std::string& toolboxPath) const;
  void matlab_methods(const std::string& classPath) const;   ///< emit method wrappers
  void matlab_static_methods(const std::string& classPath) const;   ///< emit static method wrappers
  void matlab_make_fragment(FileWriter& file,
			    const std::string& toolboxPath,
			    const std::string& mexFlags) const;   ///< emit make fragment for global make script
  void makefile_fragment(FileWriter& file) const; ///< emit makefile fragment
  std::string qualifiedName(const std::string& delim = "") const; ///< creates a namespace-qualified name, optional delimiter
};

} // \namespace wrap

