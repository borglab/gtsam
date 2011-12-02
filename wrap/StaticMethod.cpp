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
	string full_name = className + "_" + name_;
  string wrapperFile = toolboxPath + "/" + full_name + ".m";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  if(verbose_) cerr << "generating " << wrapperFile << endl;

  // generate code
  string returnType = returnVal_.matlab_returnType();
  ofs << "function " << returnType << " = " << full_name << "(";
  if (args_.size()) ofs << "," << args_.names();
  ofs << ")" << endl;
  ofs << "% usage: obj." << full_name << "(" << args_.names() << ")" << endl;
  ofs << "  error('need to compile " << full_name << ".cpp');" << endl;
  ofs << "end" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
void StaticMethod::matlab_wrapper(const string& toolboxPath,
			    const string& className, const string& nameSpace)
{
  // open destination wrapperFile
	string full_name = className + "_" + name_;
  string wrapperFile = toolboxPath + "/" + full_name + ".cpp";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  if(verbose_) cerr << "generating " << wrapperFile << endl;

  // generate code

  // header
  wrap::emit_header_comment(ofs, "//");
  ofs << "#include <wrap/matlab.h>\n";
  ofs << "#include <" << className << ".h>\n";
  if (!nameSpace.empty()) ofs << "using namespace " << nameSpace << ";" << endl;

  // call
  ofs << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  // start
  ofs << "{\n";

  // check arguments
  // extra argument obj -> nargin-1 is passed !
  // example: checkArguments("equals",nargout,nargin-1,2);
  ofs << "  checkArguments(\"" << full_name << "\",nargout,nargin," << args_.size() << ");\n";

  // unwrap arguments, see Argument.cpp
  args_.matlab_unwrap(ofs,1);

  // call method
  // example: bool result = Point2::return_field(t);
  ofs << "  ";
  if (returnVal_.returns_!="void")
    ofs << returnVal_.return_type(true,ReturnValue::pair) << " result = ";
  ofs << className  << "::" << name_ << "(" << args_.names() << ");\n";

  // wrap result
  // example: out[0]=wrap<bool>(result);
  returnVal_.wrap_result(ofs);

  // finish
  ofs << "}\n";

  // close file
  ofs.close();
}

/* ************************************************************************* */
