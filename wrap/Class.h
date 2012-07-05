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
 **/

#pragma once

#include <string>
#include <boost/bimap.hpp>

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
  std::vector<Method> methods;              ///< Class methods
  std::vector<StaticMethod> static_methods; ///< Static methods
  std::vector<std::string> namespaces;      ///< Stack of namespaces
  std::vector<std::string> using_namespaces;///< default namespaces
  std::vector<std::string> includes;        ///< header include overrides
  Constructor constructor;                  ///< Class constructors
	Deconstructor deconstructor;              ///< Deconstructor to deallocate C++ object
  bool verbose_;                            ///< verbose flag

  // And finally MATLAB code is emitted, methods below called by Module::matlab_code
  void matlab_proxy(const std::string& classFile, const std::string& wrapperName,
		FileWriter& wrapperFile, std::vector<std::string>& functionNames) const;          ///< emit proxy class
  void matlab_static_methods(const std::string& toolboxPath, const std::string& wrapperName,
		FileWriter& wrapperFile, std::vector<std::string>& functionNames) const;   ///< emit static method wrappers
  std::string qualifiedName(const std::string& delim = "") const; ///< creates a namespace-qualified name, optional delimiter

private:
	std::string pointer_constructor_fragments(FileWriter& proxyFile, FileWriter& wrapperFile, const std::string& wrapperName, int id) const;
};

} // \namespace wrap

