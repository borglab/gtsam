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
#include <list>

#include "Constructor.h"
#include "Method.h"
#include "StaticMethod.h"

namespace wrap {

/// Class has name, constructors, methods
struct Class {
  /// Constructor creates an empty class
  Class(bool verbose=true) : verbose_(verbose) {}

	// Then the instance variables are set directly by the Module constructor
  std::string name;                       ///< Class name
  std::list<Constructor> constructors;    ///< Class constructors
  std::list<Method> methods;              ///< Class methods
  std::list<StaticMethod> static_methods; ///< Static methods
  bool verbose_;                          ///< verbose flag

  // And finally MATLAB code is emitted, methods below called by Module::matlab_code
  void matlab_proxy(const std::string& classFile);          ///< emit proxy class
  void matlab_constructors(const std::string& toolboxPath,
			   const std::string& nameSpace);   ///< emit constructor wrappers
  void matlab_methods(const std::string& classPath,
			   const std::string& nameSpace);   ///< emit method wrappers
  void matlab_static_methods(const std::string& classPath,
  			   const std::string& nameSpace);   ///< emit static method wrappers
  void matlab_make_fragment(std::ofstream& ofs,
			    const std::string& toolboxPath,
			    const std::string& mexFlags);   ///< emit make fragment for global make script
};

} // \namespace wrap

