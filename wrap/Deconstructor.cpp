/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Deconstructor.ccp
 * @author Frank Dellaert
 * @author Andrew Melim
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include "utilities.h"
#include "Deconstructor.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
string Deconstructor::matlab_wrapper_name(const string& className) const {
  string str = "delete_" + className;
  return str;
}

/* ************************************************************************* */
void Deconstructor::matlab_mfile(const string& toolboxPath, const string& qualifiedMatlabName) const {

  string matlabName = matlab_wrapper_name(qualifiedMatlabName);

  // open destination m-file
  string wrapperFile = toolboxPath + "/" + matlabName + ".m";
  FileWriter file(wrapperFile, verbose_, "%");

  // generate code
  file.oss << "function result = " << matlabName << "(obj";
  if (args.size()) file.oss << "," << args.names();
  file.oss << ")" << endl;
  file.oss << "  error('need to compile " << matlabName << ".cpp');" << endl;
  file.oss << "end" << endl;

  // close file
  file.emit(true);
}

/* ************************************************************************* */
void Deconstructor::matlab_wrapper(const string& toolboxPath,
				 const string& cppClassName,
				 const string& matlabClassName,
				 const vector<string>& using_namespaces, const vector<string>& includes) const {
  string matlabName = matlab_wrapper_name(matlabClassName);

  // open destination wrapperFile
  string wrapperFile = toolboxPath + "/" + matlabName + ".cpp";
  FileWriter file(wrapperFile, verbose_, "//");

  // generate code
  //
  generateIncludes(file, name, includes);
  cout << "Generate includes " << name << endl;
  generateUsingNamespace(file, using_namespaces);
    
  file.oss << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])" << endl;
  file.oss << "{" << endl;
  //Deconstructor takes 1 arg, the mxArray obj
  file.oss << "  checkArguments(\"" << matlabName << "\",nargout,nargin," << "1" << ");" << endl;
  file.oss << "  delete_shared_ptr< " << cppClassName << " >(in[0],\"" << matlabClassName << "\");" << endl;
  file.oss << "}" << endl;

  // close file
  file.emit(true);
}

/* ************************************************************************* */
