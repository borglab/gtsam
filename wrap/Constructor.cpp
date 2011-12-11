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
void Constructor::matlab_proxy_fragment(ofstream& ofs, const string& className) const {
  ofs << "      if nargin == " << args.size() << ", obj.self = " 
      << matlab_wrapper_name(className) << "(";
  bool first = true;
  for(size_t i=0;i<args.size();i++) {
    if (!first) ofs << ",";
    ofs << "varargin{" << i+1 << "}";
    first=false;
  }
  ofs << "); end" << endl;
}

/* ************************************************************************* */
void Constructor::matlab_mfile(const string& toolboxPath, const string& qualifiedMatlabName) const {

  string matlabName = matlab_wrapper_name(qualifiedMatlabName);

  // open destination m-file
  string wrapperFile = toolboxPath + "/" + matlabName + ".m";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  if(verbose_) cerr << "generating " << wrapperFile << endl;

  // generate code
  generateHeaderComment(ofs, "%");
  ofs << "function result = " << matlabName << "(obj";
  if (args.size()) ofs << "," << args.names();
  ofs << ")" << endl;
  ofs << "  error('need to compile " << matlabName << ".cpp');" << endl;
  ofs << "end" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
void Constructor::matlab_wrapper(const string& toolboxPath,
				 const string& cppClassName,
				 const string& matlabClassName,
				 const vector<string>& using_namespaces, const vector<string>& includes) const {
  string matlabName = matlab_wrapper_name(matlabClassName);

  // open destination wrapperFile
  string wrapperFile = toolboxPath + "/" + matlabName + ".cpp";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  if(verbose_) cerr << "generating " << wrapperFile << endl;

  // generate code
  generateHeaderComment(ofs, "//");
  generateIncludes(ofs, name, includes);
  generateUsingNamespace(ofs, using_namespaces);

  ofs << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])" << endl;
  ofs << "{" << endl;
  ofs << "  checkArguments(\"" << matlabName << "\",nargout,nargin," << args.size() << ");" << endl;
  args.matlab_unwrap(ofs); // unwrap arguments
  ofs << "  " << cppClassName << "* self = new " << cppClassName << "(" << args.names() << ");" << endl; // need qualified name, delim: "::"
  ofs << "  out[0] = wrap_constructed(self,\"" << matlabClassName << "\");" << endl; // need matlab qualified name
  ofs << "}" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
