/**
 * file: Module.h
 * brief: describes module to be wrapped
 * Author: Frank Dellaert
 **/

#pragma once

#include <string>
#include <list>

#include "Class.h"

// A module has classes
struct Module {
  std::string name;
  std::list<Class> classes;

  /**
   * constructor that parses interface file
   */
  Module(const std::string& interfacePath, 
	 const std::string& moduleName);

  /**
   *  MATLAB code generation:
   */
  void matlab_code(const std::string& path, 
		   const std::string& nameSpace, 
		   const std::string& mexFlags);
};

