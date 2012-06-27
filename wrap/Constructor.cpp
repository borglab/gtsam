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
 * @author Andrew Melim
 **/

#include <iostream>
#include <fstream>
#include <algorithm>

#include <boost/foreach.hpp>

#include "utilities.h"
#include "Constructor.h"

using namespace std;
using namespace wrap;


/* ************************************************************************* */
string Constructor::matlab_wrapper_name(const string& className) const {
  string str = "new_" + className + "_";
  return str;
}

/* ************************************************************************* */
void Constructor::matlab_proxy_fragment(FileWriter& file, 
        const string& className, const int id, const ArgumentList args) const {
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
  file.oss << "), obj.self = " << matlab_wrapper_name(className) << "(" << "0," << id;
  // emit constructor arguments
  for(size_t i=0;i<nrArgs;i++) {
    file.oss << ",";
    file.oss << "varargin{" << i+1 << "}";
  }
  file.oss << "); end" << endl;
}

/* ************************************************************************* */
void Constructor::matlab_mfile(const string& toolboxPath, const string& qualifiedMatlabName, const ArgumentList args) const {

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
void Constructor::matlab_wrapper(const string& toolboxPath,
				 const string& cppClassName,
				 const string& matlabClassName,
				 const vector<string>& using_namespaces, 
				 const vector<string>& includes) const {
  string matlabName = matlab_wrapper_name(matlabClassName);

  // open destination wrapperFile
  string wrapperFile = toolboxPath + "/" + matlabName + ".cpp";
  FileWriter file(wrapperFile, verbose_, "//");

  // generate code
  generateIncludes(file, name, includes);
  generateUsingNamespace(file, using_namespaces);

  //Typedef boost::shared_ptr
  file.oss << "typedef boost::shared_ptr<"  << cppClassName  << "> Shared;" << endl;
  file.oss << endl;

  //Generate collector
  file.oss << "static std::set<Shared*> collector;" << endl;
  file.oss << endl;

  //Generate the destructor function
  file.oss << "struct Destruct" << endl;
  file.oss << "{" << endl;
  file.oss << "  void operator() (Shared* p)" << endl;
  file.oss << "  {" << endl;
  file.oss << "    collector.erase(p);" << endl;
  file.oss << "  }" << endl;
  file.oss << "};" << endl;
  file.oss << endl;

  //Generate cleanup function
  file.oss << "void cleanup(void) {" << endl;
  file.oss << "  std::for_each( collector.begin(), collector.end(), Destruct() );" << endl;
  file.oss << "}" << endl;

  file.oss << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])" << endl;
  file.oss << "{" << endl;
  //Cleanup function callback
  file.oss << "  mexAtExit(cleanup);" << endl;
  file.oss << endl;
  file.oss << "  const mxArray* input = in[0];" << endl;
  file.oss << "  Shared* self = *(Shared**) mxGetData(input);" << endl;
  file.oss << endl;
  file.oss << "  if(self) {" << endl;
  file.oss << "    if(nargin > 1) {" << endl;
  file.oss << "      collector.insert(self);" << endl;
  //TODO: Add verbosity flag
  file.oss << "      std::cout << \"Collected\" << collector.size() << std::endl;" << endl;
  file.oss << "    }" << endl;
  file.oss << "    else if(collector.erase(self))" << endl;
  file.oss << "      delete self;" << endl;
  file.oss << "  } else {" << endl;
  file.oss << "    int nc = unwrap<int>(in[1]);" << endl;

  int i = 0;
  BOOST_FOREACH(ArgumentList al, args_list)
  {
    file.oss << "    if(nc == " << i <<") {" << endl;
    al.matlab_unwrap(file, 2); // unwrap arguments, start at 1
    file.oss << "      self = new Shared(new " << cppClassName << "(" << al.names() << "));" << endl;
    file.oss << "    }" << endl;
    i++;
  }
  
  //file.oss << "    self = construct(nc, in);" << endl;
  file.oss << "    collector.insert(self);" << endl;
  file.oss << "    std::cout << \"constructed \" << self << \", size=\" << collector.size() << std::endl;" << endl;
  file.oss << "    out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);" << endl;
  file.oss << "    *reinterpret_cast<Shared**> (mxGetPr(out[0])) = self;" << endl;
  file.oss << "  }" << endl;

  file.oss << "}" << endl;

  // close file
  file.emit(true);
}

/* ************************************************************************* */
