/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Constructor.h
 * @brief class describing a constructor + code generation
 * @author Frank Dellaert
 **/

#pragma once

#include <string>
#include <list>

#include "Argument.h"

// Constructor class
struct Constructor {
  ArgumentList args;
  bool verbose_;

  Constructor(bool verbose=true) : verbose_(verbose) {}

  // MATLAB code generation
  // toolboxPath is main toolbox directory, e.g., ../matlab
  // classFile is class proxy file, e.g., ../matlab/@Point2/Point2.m

  std::string matlab_wrapper_name(const std::string& className);                     // wrapper name
  void matlab_proxy_fragment(std::ofstream& ofs, const std::string& className);      // proxy class fragment
  void matlab_mfile  (const std::string& toolboxPath, const std::string& className); // m-file
  void matlab_wrapper(const std::string& toolboxPath, 
		      const std::string& className, 
		      const std::string& nameSpace); // wrapper
};

