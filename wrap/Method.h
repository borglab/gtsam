/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Method.h
 * @brief describes and generates code for methods
 * @author Frank Dellaert
 **/

#pragma once

#include <string>
#include <list>

#include "Argument.h"
#include "ReturnValue.h"

namespace wrap {

/// Method class
struct Method {

	/// Constructor creates empty object
	Method(bool verbose = true) :
			verbose_(verbose) {}

	// Then the instance variables are set directly by the Module constructor
	bool verbose_;
	bool is_const_;
	std::string name_;
	ArgumentList args_;
	ReturnValue returnVal_;

	// MATLAB code generation
	// classPath is class directory, e.g., ../matlab/@Point2

	void matlab_mfile(const std::string& classPath); ///< m-file
	void matlab_wrapper(const std::string& classPath,
			const std::string& className, const std::string& nameSpace); ///< wrapper
};

} // \namespace wrap

