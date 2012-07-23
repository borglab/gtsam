/**
 * @file GlobalFunction.cpp
 *
 * @date Jul 22, 2012
 * @author Alex Cunningham
 */

#include "GlobalFunction.h"

namespace wrap {

/* ************************************************************************* */
void GlobalFunction::addOverload(bool verbose, const std::string& name,
		const ArgumentList& args, const ReturnValue& retVal, const StrVec& ns_stack) {
	this->verbose_ = verbose;
	this->name = name;
	this->argLists.push_back(args);
	this->returnVals.push_back(retVal);
	this->namespaces.push_back(ns_stack);
}

/* ************************************************************************* */


} // \namespace wrap




