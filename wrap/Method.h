/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * file: Method.h
 * brief: describes and generates code for methods
 * Author: Frank Dellaert
 **/

#pragma once

#include <string>
#include <list>

#include "Argument.h"

// Method class
struct Method {
  bool is_const;
  ArgumentList args;
  std::string returns, returns2, name;
  bool returns_ptr, returns_ptr2, returns_pair;
  bool verbose_;

  Method(bool verbose=true) : returns_ptr(false), returns_ptr2(false), returns_pair(false), verbose_(verbose) {}

  enum pairing {arg1, arg2, pair};
  std::string return_type(bool add_ptr, pairing p);

  // MATLAB code generation
  // classPath is class directory, e.g., ../matlab/@Point2

  void matlab_mfile  (const std::string& classPath); // m-file
  void matlab_wrapper(const std::string& classPath, 
		      const std::string& className, 
		      const std::string& nameSpace); // wrapper
};

