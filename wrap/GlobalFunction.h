/**
 * @file GlobalFunction.h
 *
 * @brief Implements codegen for a global function wrapped in matlab
 * 
 * @date Jul 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include "Argument.h"
#include "ReturnValue.h"

namespace wrap {

struct GlobalFunction {

	typedef std::vector<std::string> StrVec;

	bool verbose_;
	std::string name;

	// each overload, regardless of namespace
	std::vector<ArgumentList> argLists;       ///< arugments for each overload
	std::vector<ReturnValue> returnVals;      ///< returnVals for each overload
	std::vector<StrVec> namespaces;           ///< Stack of namespaces

	// Constructor only used in Module
	GlobalFunction(bool verbose = true) : verbose_(verbose) {}

	// Used to reconstruct
	GlobalFunction(const std::string& name_, bool verbose = true)
	: verbose_(verbose), name(name_) {}

	// adds an overloaded version of this function
	void addOverload(bool verbose, const std::string& name,
		const ArgumentList& args, const ReturnValue& retVal, const StrVec& ns_stack);

	// codegen function called from Module to build the cpp and matlab versions of the function
  void matlab_proxy(const std::string& toolboxPath, const std::string& wrapperName,
  		const TypeAttributesTable& typeAttributes,	FileWriter& wrapperFile,
  		std::vector<std::string>& functionNames) const;

private:

  // Creates a single global function - all in same namespace
  void generateSingleFunction(const std::string& toolboxPath, const std::string& wrapperName,
  		const TypeAttributesTable& typeAttributes,	FileWriter& wrapperFile,
  		std::vector<std::string>& functionNames) const;

};

} // \namespace wrap




