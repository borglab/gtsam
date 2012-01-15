/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Constructor.ccp
 * @author Frank Dellaert
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include "utilities.h"
#include "Constructor.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
string Constructor::matlab_wrapper_name(const string& className) const {
  string str = "new_" + className + "_" + args.signature();
  return str;
}

/* ************************************************************************* */
void Constructor::matlab_proxy_fragment(FileWriter& file, const string& className) const {
	size_t nrArgs = args.size();
	// check for number of arguments...
  file.oss << "      if (nargin == " << nrArgs;
  if (nrArgs>0) file.oss << " && ";
	// ...and their types
  bool first = true;
  for(size_t i=0;i<nrArgs;i++) {
    if (!first) file.oss << " && ";
    file.oss << "isa(varargin{" << i+1 << "},'" << args[i].matlabClass() << "')";
    first=false;
  }
  // emit code for calling constructor
  file.oss << "), obj.self = " << matlab_wrapper_name(className) << "(";
  // emit constructor arguments
  first = true;
  for(size_t i=0;i<nrArgs;i++) {
    if (!first) file.oss << ",";
    file.oss << "varargin{" << i+1 << "}";
    first=false;
  }
  file.oss << "); end" << endl;
}

/* ************************************************************************* */
void Constructor::matlab_mfile(const string& toolboxPath, const string& qualifiedMatlabName) const {

  string matlabName = matlab_wrapper_name(qualifiedMatlabName);

  // open destination m-file
  string wrapperFile = toolboxPath + "/" + matlabName + ".m";
  FileWriter file(wrapperFile, "%");
//  if(!file) throw CantOpenFile(wrapperFile);
  if(verbose_) cerr << "generating " << wrapperFile << endl;

  // generate code
//  generateHeaderComment(file, "%");
  file.oss << "function result = " << matlabName << "(obj";
  if (args.size()) file.oss << "," << args.names();
  file.oss << ")" << endl;
  file.oss << "  error('need to compile " << matlabName << ".cpp');" << endl;
  file.oss << "end" << endl;

  // close file
  file.emit(true);
}

/* ************************************************************************* */
void Constructor::matlab_wrapper(const string& toolboxPath,
				 const string& cppClassName,
				 const string& matlabClassName,
				 const vector<string>& using_namespaces, const vector<string>& includes) const {
  string matlabName = matlab_wrapper_name(matlabClassName);

  // open destination wrapperFile
  string wrapperFile = toolboxPath + "/" + matlabName + ".cpp";
  FileWriter file(wrapperFile, "//");
//  if(!file) throw CantOpenFile(wrapperFile);
  if(verbose_) cerr << "generating " << wrapperFile << endl;

  // generate code
//  generateHeaderComment(file, "//");
  generateIncludes(file, name, includes);
  generateUsingNamespace(file, using_namespaces);

  file.oss << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])" << endl;
  file.oss << "{" << endl;
  file.oss << "  checkArguments(\"" << matlabName << "\",nargout,nargin," << args.size() << ");" << endl;
  args.matlab_unwrap(file); // unwrap arguments
  file.oss << "  " << cppClassName << "* self = new " << cppClassName << "(" << args.names() << ");" << endl; // need qualified name, delim: "::"
  file.oss << "  out[0] = wrap_constructed(self,\"" << matlabClassName << "\");" << endl; // need matlab qualified name
  file.oss << "}" << endl;

  // close file
  file.emit(true);
}

/* ************************************************************************* */
