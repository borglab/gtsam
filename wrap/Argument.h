/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Argument.h
 * @brief arguments to constructors and methods
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#pragma once

#include <string>
#include <vector>

#include "FileWriter.h"

namespace wrap {

/// Argument class
struct Argument {
	bool is_const, is_ref, is_ptr;
	std::string type;
	std::string name;
	std::vector<std::string> namespaces;

	Argument() :
			is_const(false), is_ref(false), is_ptr(false) {
	}

	/// return MATLAB class for use in isa(x,class)
	std::string matlabClass(const std::string& delim = "") const;

	/// adds namespaces to type
	std::string qualifiedType(const std::string& delim = "") const;

	/// MATLAB code generation, MATLAB to C++
	void matlab_unwrap(FileWriter& file, const std::string& matlabName) const;
};

/// Argument list is just a container with Arguments
struct ArgumentList: public std::vector<Argument> {

	/// create a comma-separated string listing all argument types (not used)
	std::string types() const;

	/// create a short "signature" string
	std::string signature() const;

	/// create a comma-separated string listing all argument names, used in m-files
	std::string names() const;

	// MATLAB code generation:

	/**
	 * emit code to unwrap arguments
	 * @param file output stream
	 * @param start initial index for input array, set to 1 for method
	 */
	void matlab_unwrap(FileWriter& file, int start = 0) const; // MATLAB to C++

};

} // \namespace wrap

