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
  FileWriter file(wrapperFile, verbose_, "%");

  // generate code
  string returnType = returnVal.matlab_returnType();
  file.oss << "% " << returnType << " = obj." << name << "(" << args.names() << ")" << endl;
  file.oss << "function " << returnType << " = " << name << "(obj";
  if (args.size()) file.oss << "," << args.names();
  file.oss << ")" << endl;
  file.oss << "  error('need to compile " << name << ".cpp');" << endl;
  file.oss << "end" << endl;

  // close file
  file.emit(false);
}

/* ************************************************************************* */
void Method::matlab_wrapper(const string& classPath, 
			    const string& className,
			    const string& cppClassName,
			    const string& matlabClassName,
			    const vector<string>& using_namespaces, const std::vector<std::string>& includes) const {
  // open destination wrapperFile
  string wrapperFile = classPath + "/" + name + ".cpp";
  FileWriter file(wrapperFile, verbose_, "//");

  // generate code

  // header
  generateIncludes(file, className, includes);
  generateUsingNamespace(file, using_namespaces);

  if(returnVal.isPair)
  {
    file.oss << "typedef boost::shared_ptr<"  << returnVal.qualifiedType1("::")  << "> Shared" <<  returnVal.type1 << ";"<< endl;
    file.oss << "typedef boost::shared_ptr<"  << returnVal.qualifiedType2("::")  << "> Shared" <<  returnVal.type2 << ";"<< endl;
  }
  else
    file.oss << "typedef boost::shared_ptr<"  << returnVal.qualifiedType1("::")  << "> Shared" <<  returnVal.type1 << ";"<< endl;

  file.oss << "typedef boost::shared_ptr<"  << cppClassName  << "> Shared;" << endl;
  // call
  file.oss << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  // start
  file.oss << "{\n";

  // check arguments
  // extra argument obj -> nargin-1 is passed !
  // example: checkArguments("equals",nargout,nargin-1,2);
  file.oss << "  checkArguments(\"" << name << "\",nargout,nargin-1," << args.size() << ");\n";

  // get class pointer
  // example: shared_ptr<Test> = unwrap_shared_ptr< Test >(in[0], "Test");
  file.oss << "  mxArray* mxh = mxGetProperty(in[0],0,\"self\");" << endl;
  file.oss << "  Shared* self = *reinterpret_cast<Shared**> (mxGetPr(mxh));" << endl; 
  file.oss << "  Shared obj = *self;" << endl;
  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(file,1);

  // call method
  // example: bool result = self->return_field(t);
  file.oss << "  ";
  if (returnVal.type1!="void")
    file.oss << returnVal.return_type(true,ReturnValue::pair) << " result = ";
  file.oss << "obj->" << name << "(" << args.names() << ");\n";

  // wrap result
  // example: out[0]=wrap<bool>(result);
  returnVal.wrap_result(file);

  // finish
  file.oss << "}\n";

  // close file
  file.emit(true);
}

/* ************************************************************************* */
