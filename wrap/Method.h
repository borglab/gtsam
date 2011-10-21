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

/// Method class
struct Method {

	/// Constructor creates empty object
	Method(bool verbose = true) :
			returns_ptr_(false), returns_ptr2_(false), returns_pair_(false), verbose_(
					verbose) {
	}

	// Then the instance variables are set directly by the Module constructor
	bool is_const_;
	ArgumentList args_;
	std::string returns_, returns2_, name_;
	bool returns_ptr_, returns_ptr2_, returns_pair_;
	bool verbose_;

	enum pairing {
		arg1, arg2, pair
	};
	std::string return_type(bool add_ptr, pairing p);

	// MATLAB code generation
	// classPath is class directory, e.g., ../matlab/@Point2

	void matlab_mfile(const std::string& classPath); ///< m-file
	void matlab_wrapper(const std::string& classPath,
			const std::string& className, const std::string& nameSpace); ///< wrapper
};

