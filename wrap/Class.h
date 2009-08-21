/**
 * file: Class.h
 * brief: describe the C++ class that is being wrapped
 * Author: Frank Dellaert
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

