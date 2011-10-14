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

// Class has name, constructors, methods
struct Class {
  std::string name;
  std::list<Constructor> constructors;
  std::list<Method> methods;
  bool verbose_;

  Class(bool verbose=true) : verbose_(verbose) {}

  // MATLAB code generation:
  void matlab_proxy(const std::string& classFile);          // proxy class
  void matlab_constructors(const std::string& toolboxPath, 
			   const std::string& nameSpace);   // constructor wrappers
  void matlab_methods(const std::string& classPath, 
			   const std::string& nameSpace);   // method wrappers
  void matlab_make_fragment(std::ofstream& ofs, 
			    const std::string& toolboxPath, 
			    const std::string& mexFlags);   // make fragment
};

