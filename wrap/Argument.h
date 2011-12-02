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
 **/

#pragma once

#include <string>
#include <list>

namespace wrap {

/// Argument class
struct Argument {
	bool is_const, is_ref, is_ptr;
	std::string type;
	std::string name;
	Argument() :
			is_const(false), is_ref(false), is_ptr(false) {
	}

	/// MATLAB code generation, MATLAB to C++
	void matlab_unwrap(std::ofstream& ofs, const std::string& matlabName);
};

/// Argument list
struct ArgumentList: public std::list<Argument> {
	std::list<Argument> args;
	std::string types();
	std::string signature();
	std::string names();

	// MATLAB code generation:

	/**
	 * emit code to unwrap arguments
	 * @param ofs output stream
	 * @param start initial index for input array, set to 1 for method
	 */
	void matlab_unwrap(std::ofstream& ofs, int start = 0); // MATLAB to C++
};

} // \namespace wrap

