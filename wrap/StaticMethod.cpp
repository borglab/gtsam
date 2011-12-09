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
void StaticMethod::matlab_mfile(const string& toolboxPath, const string& className) {

  // open destination m-file
	string full_name = className + "_" + name;
  string wrapperFile = toolboxPath + "/" + full_name + ".m";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  if(verbose) cerr << "generating " << wrapperFile << endl;

  // generate code
  string returnType = returnVal.matlab_returnType();
  ofs << "function " << returnType << " = " << full_name << "(";
  if (args.size()) ofs << args.names();
  ofs << ")" << endl;
  ofs << "% usage: x = " << full_name << "(" << args.names() << ")" << endl;
  ofs << "  error('need to compile " << full_name << ".cpp');" << endl;
  ofs << "end" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
void StaticMethod::matlab_wrapper(const string& toolboxPath, const string& className,
		const string& matlabClassName, const string& cppClassName,
		const vector<string>& using_namespaces,
		const std::vector<std::string>& includes)
{
  // open destination wrapperFile
	string full_name = matlabClassName + "_" + name;
  string wrapperFile = toolboxPath + "/" + full_name + ".cpp";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  if(verbose) cerr << "generating " << wrapperFile << endl;

  // generate code

  // header
  wrap::emit_header_comment(ofs, "//");
  generateIncludes(ofs, className, includes);
  generateUsingNamespace(ofs, using_namespaces);

  // call
  ofs << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  // start
  ofs << "{\n";

  // check arguments
  // NOTE: for static functions, there is no object passed
  ofs << "  checkArguments(\"" << full_name << "\",nargout,nargin," << args.size() << ");\n";

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(ofs,0); // We start at 0 because there is no self object

  ofs << "  ";

  // call method with default type
  if (returnVal.type1!="void")
    ofs << returnVal.return_type(true,ReturnValue::pair) << " result = ";
  ofs << cppClassName  << "::" << name << "(" << args.names() << ");\n";

  // wrap result
  // example: out[0]=wrap<bool>(result);
  returnVal.wrap_result(ofs);

  // finish
  ofs << "}\n";

  // close file
  ofs.close();
}

/* ************************************************************************* */
