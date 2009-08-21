/**
 * file: Constructor.ccp
 * Author: Frank Dellaert
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include "utilities.h"
#include "Constructor.h"

using namespace std;

/* ************************************************************************* */
string Constructor::matlab_wrapper_name(const string& className) {
  string str = "new_" + className + "_" + args.signature();
  return str;
}

/* ************************************************************************* */
void Constructor::matlab_proxy_fragment(ofstream& ofs, const string& className) {
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
void Constructor::matlab_mfile(const string& toolboxPath, const string& className) {

  string name = matlab_wrapper_name(className);

  // open destination m-file
  string wrapperFile = toolboxPath + "/" + name + ".m";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  cerr << "generating " << wrapperFile << endl;

  // generate code
  emit_header_comment(ofs, "%");
  ofs << "function result = " << name << "(obj";
  if (args.size()) ofs << "," << args.names();
  ofs << ")" << endl;
  ofs << "  error('need to compile " << name << ".cpp');" << endl;
  ofs << "end" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
void Constructor::matlab_wrapper(const string& toolboxPath, 
				 const string& className,
				 const string& nameSpace) 
{

  string name = matlab_wrapper_name(className);

  // open destination wrapperFile
  string wrapperFile = toolboxPath + "/" + name + ".cpp";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  cerr << "generating " << wrapperFile << endl;

  // generate code
  emit_header_comment(ofs, "//");
  ofs << "#include <wrap/matlab.h>" << endl;
  ofs << "#include <" << className << ".h>" << endl;
  if (!nameSpace.empty()) ofs << "using namespace " << nameSpace << ";" << endl;
  ofs << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])" << endl;
  ofs << "{" << endl;
  ofs << "  checkArguments(\"" << name << "\",nargout,nargin," << args.size() << ");" << endl;
  args.matlab_unwrap(ofs); // unwrap arguments
  ofs << "  " << className << "* self = new " << className << "(" << args.names() << ");" << endl;
  ofs << "  out[0] = wrap_constructed(self,\"" << className << "\");" << endl;
  ofs << "}" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
