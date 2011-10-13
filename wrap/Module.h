/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
  bool verbose_;

  /**
   * constructor that parses interface file
   */
  Module(const std::string& interfacePath, 
	 const std::string& moduleName,
	 bool verbose=true);

  /**
   *  MATLAB code generation:
   */
  void matlab_code(const std::string& path, 
		   const std::string& nameSpace, 
		   const std::string& mexFlags);
};

