/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Method.ccp
 * @author Frank Dellaert
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include "StaticMethod.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void StaticMethod::matlab_mfile(const string& toolboxPath, const string& className) const {

  // open destination m-file
	string full_name = className + "_" + name;
  string wrapperFile = toolboxPath + "/" + full_name + ".m";
  FileWriter file(wrapperFile, verbose);

  // generate code
  string returnType = returnVal.matlab_returnType();
  file.oss << "function " << returnType << " = " << full_name << "(";
  if (args.size()) file.oss << args.names();
  file.oss << ")" << endl;
  file.oss << "% usage: x = " << full_name << "(" << args.names() << ")" << endl;
  file.oss << "  error('need to compile " << full_name << ".cpp');" << endl;
  file.oss << "end" << endl;

  // close file
  file.emit(false);
}

/* ************************************************************************* */
void StaticMethod::matlab_wrapper(const string& toolboxPath, const string& className,
		const string& matlabClassName, const string& cppClassName,
		const vector<string>& using_namespaces,
		const vector<string>& includes) const {
  // open destination wrapperFile
	string full_name = matlabClassName + "_" + name;
  string wrapperFile = toolboxPath + "/" + full_name + ".cpp";
  FileWriter file(wrapperFile, verbose, "//");

  // generate code

  // header
  generateIncludes(file, className, includes);
  generateUsingNamespace(file, using_namespaces);

  // call
  file.oss << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  // start
  file.oss << "{\n";

  // check arguments
  // NOTE: for static functions, there is no object passed
  file.oss << "  checkArguments(\"" << full_name << "\",nargout,nargin," << args.size() << ");\n";

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(file,0); // We start at 0 because there is no self object

  file.oss << "  ";

  // call method with default type
  if (returnVal.type1!="void")
    file.oss << returnVal.return_type(true,ReturnValue::pair) << " result = ";
  file.oss << cppClassName  << "::" << name << "(" << args.names() << ");\n";

  // wrap result
  // example: out[0]=wrap<bool>(result);
  returnVal.wrap_result(file);

  // finish
  file.oss << "}\n";

  // close file
  file.emit(true);
}

/* ************************************************************************* */
