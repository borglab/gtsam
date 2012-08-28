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
 * @author Richard Roberts
 **/

#pragma once

#include <string>
#include <list>

#include "Argument.h"
#include "ReturnValue.h"
#include "TypeAttributesTable.h"
#include <boost/algorithm/string.hpp>

namespace wrap {

/// StaticMethod class
struct StaticMethod {

	/// Constructor creates empty object
	StaticMethod(bool verbosity = true) :
			verbose(verbosity) {}

	// Then the instance variables are set directly by the Module constructor
	bool verbose;
	std::string name;
	std::vector<ArgumentList> argLists;
	std::vector<ReturnValue> returnVals;

	// The first time this function is called, it initializes the class members
	// with those in rhs, but in subsequent calls it adds additional argument
	// lists as function overloads.
	void addOverload(bool verbose, const std::string& name,
		const ArgumentList& args, const ReturnValue& retVal);

	// MATLAB code generation
	// classPath is class directory, e.g., ../matlab/@Point2
	void proxy_wrapper_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
		const std::string& cppClassName, const std::string& matlabQualName, const std::string& matlabUniqueName,
		const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
		std::vector<std::string>& functionNames) const;

private:
	std::string wrapper_fragment(FileWriter& file,
	    const std::string& cppClassName,
	    const std::string& matlabUniqueName,
			int overload,
			int id,
			const TypeAttributesTable& typeAttributes) const; ///< cpp wrapper
};

} // \namespace wrap

