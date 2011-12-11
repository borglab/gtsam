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

#include "Method.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Method::matlab_mfile(const string& classPath) const {

  // open destination m-file
  string wrapperFile = classPath + "/" + name + ".m";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  if(verbose_) cerr << "generating " << wrapperFile << endl;

  // generate code
  string returnType = returnVal.matlab_returnType();
  ofs << "function " << returnType << " = " << name << "(obj";
  if (args.size()) ofs << "," << args.names();
  ofs << ")" << endl;
  ofs << "% usage: obj." << name << "(" << args.names() << ")" << endl;
  ofs << "  error('need to compile " << name << ".cpp');" << endl;
  ofs << "end" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
void Method::matlab_wrapper(const string& classPath, 
			    const string& className,
			    const string& cppClassName,
			    const string& matlabClassName,
			    const vector<string>& using_namespaces, const std::vector<std::string>& includes) const {
  // open destination wrapperFile
  string wrapperFile = classPath + "/" + name + ".cpp";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  if(verbose_) cerr << "generating " << wrapperFile << endl;

  // generate code

  // header
  generateHeaderComment(ofs, "//");
  generateIncludes(ofs, className, includes);
  generateUsingNamespace(ofs, using_namespaces);

  // call
  ofs << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  // start
  ofs << "{\n";

  // check arguments
  // extra argument obj -> nargin-1 is passed !
  // example: checkArguments("equals",nargout,nargin-1,2);
  ofs << "  checkArguments(\"" << name << "\",nargout,nargin-1," << args.size() << ");\n";

  // get class pointer
  // example: shared_ptr<Test> = unwrap_shared_ptr< Test >(in[0], "Test");
  ofs << "  shared_ptr<" << cppClassName << "> self = unwrap_shared_ptr< " << cppClassName
      << " >(in[0],\"" << matlabClassName << "\");" << endl;

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(ofs,1);

  // call method
  // example: bool result = self->return_field(t);
  ofs << "  ";
  if (returnVal.type1!="void")
    ofs << returnVal.return_type(true,ReturnValue::pair) << " result = ";
  ofs << "self->" << name << "(" << args.names() << ");\n";

  // wrap result
  // example: out[0]=wrap<bool>(result);
  returnVal.wrap_result(ofs);

  // finish
  ofs << "}\n";

  // close file
  ofs.close();
}

/* ************************************************************************* */
