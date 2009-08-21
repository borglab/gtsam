/**
 * file: Method.ccp
 * Author: Frank Dellaert
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include "Method.h"
#include "utilities.h"

using namespace std;

/* ************************************************************************* */
// auxiliary function to wrap an argument into a shared_ptr template
/* ************************************************************************* */
string maybe_shared_ptr(bool add, const string& type) {
  string str = add? "shared_ptr<" : "";
  str += type;
  if (add) str += ">";
  return str;
}

/* ************************************************************************* */
string Method::return_type(bool add_ptr, pairing p) {
  if (p==pair && returns_pair) {
    string str = "pair< " + 
      maybe_shared_ptr(add_ptr && returns_ptr, returns ) + ", " + 
      maybe_shared_ptr(add_ptr && returns_ptr, returns2) + " >";
    return str;
  } else
    return maybe_shared_ptr(add_ptr && returns_ptr, (p==arg2)? returns2 : returns);
}

/* ************************************************************************* */
void Method::matlab_mfile(const string& classPath) {

  // open destination m-file
  string wrapperFile = classPath + "/" + name + ".m";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  cerr << "generating " << wrapperFile << endl;

  // generate code
  emit_header_comment(ofs, "%");
  ofs << "% usage: obj." << name << "(" << args.names() << ")" << endl;
  string returnType = returns_pair? "[first,second]" : "result";
  ofs << "function " << returnType << " = " << name << "(obj";
  if (args.size()) ofs << "," << args.names();
  ofs << ")" << endl;
  ofs << "  error('need to compile " << name << ".cpp');" << endl;
  ofs << "end" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
void Method::matlab_wrapper(const string& classPath, 
			    const string& className,
			    const string& nameSpace) 
{
  // open destination wrapperFile
  string wrapperFile = classPath + "/" + name + ".cpp";
  ofstream ofs(wrapperFile.c_str());
  if(!ofs) throw CantOpenFile(wrapperFile);
  cerr << "generating " << wrapperFile << endl;

  // generate code

  // header
  emit_header_comment(ofs, "//");
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
  ofs << "  checkArguments(\"" << name << "\",nargout,nargin-1," << args.size() << ");\n";

  // get class pointer
  // example: shared_ptr<Test> = unwrap_shared_ptr< Test >(in[0], "Test");
  ofs << "  shared_ptr<" << className << "> self = unwrap_shared_ptr< " << className 
      << " >(in[0],\"" << className << "\");" << endl;

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(ofs,1);

  // call method
  // example: bool result = self->return_field(t);
  ofs << "  ";
  if (returns!="void") 
    ofs << return_type(true,pair) << " result = ";
  ofs << "self->" << name << "(" << args.names() << ");\n";

  // wrap result
  // example: out[0]=wrap<bool>(result);
  if (returns_pair) {
    if (returns_ptr)
      ofs << "  out[0] = wrap_shared_ptr(result.first,\"" << returns << "\");\n";
    else
      ofs << "  out[0] = wrap< " << return_type(true,arg1) << " >(result.first);\n";
    if (returns_ptr2)
      ofs << "  out[1] = wrap_shared_ptr(result.second,\"" << returns2 << "\");\n";
    else
      ofs << "  out[1] = wrap< " << return_type(true,arg2) << " >(result.second);\n";
  } 
  else if (returns_ptr)
    ofs << "  out[0] = wrap_shared_ptr(result,\"" << returns << "\");\n";
  else if (returns!="void")
    ofs << "  out[0] = wrap< " << return_type(true,arg1) << " >(result);\n";

  // finish
  ofs << "}\n";

  // close file
  ofs.close();
}

/* ************************************************************************* */
