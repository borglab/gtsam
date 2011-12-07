/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file StaticMethod.h
 * @brief describes and generates code for static methods
 * @author Frank Dellaert
 * @author Alex Cunningham
 **/

#pragma once

#include <string>
#include <list>

#include "Argument.h"
#include "ReturnValue.h"

namespace wrap {

/// StaticMethod class
struct StaticMethod {

	/// Constructor creates empty object
	StaticMethod(bool verbosity = true) :
			verbose(verbosity) {}

	// Then the instance variables are set directly by the Module constructor
	bool verbose;
	std::string name;
	ArgumentList args;
	ReturnValue returnVal;

	// MATLAB code generation
	// toolboxPath is the core toolbox directory, e.g., ../matlab
	// NOTE: static functions are not inside the class, and
	// are created with [ClassName]_[FunctionName]() format

	void matlab_mfile(const std::string& toolboxPath, const std::string& className); ///< m-file
	void matlab_wrapper(const std::string& toolboxPath,
			const std::string& className, const std::string& nameSpace); ///< wrapper
};

} // \namespace wrap

