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
#include <vector>

#include "Argument.h"

namespace wrap {

// Constructor class
struct Constructor {

	/// Constructor creates an empty class
	Constructor(bool verbose = true) :
			verbose_(verbose) {
	}

	// Then the instance variables are set directly by the Module constructor
	ArgumentList args;
	std::string name;
	bool verbose_;

	// MATLAB code generation
	// toolboxPath is main toolbox directory, e.g., ../matlab
	// classFile is class proxy file, e.g., ../matlab/@Point2/Point2.m

	/// wrapper name
	std::string matlab_wrapper_name(const std::string& className);

	/// proxy class fragment
	void matlab_proxy_fragment(std::ofstream& ofs, const std::string& className);

	/// m-file
	void matlab_mfile(const std::string& toolboxPath,
			const std::string& qualifiedMatlabName);

	/// cpp wrapper
	void matlab_wrapper(const std::string& toolboxPath,
			 const std::string& cppClassName,
			 const std::string& matlabClassName,
			 const std::string& nameSpace, const std::vector<std::string>& includes);
};

} // \namespace wrap

