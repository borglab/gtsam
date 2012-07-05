/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file StaticMethod.ccp
 * @author Frank Dellaert
 * @author Andrew Melim
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include "StaticMethod.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void StaticMethod::proxy_fragment(const string& toolboxPath, const string& matlabClassName, const std::string& wrapperName, const int id) const {

	const string full_name = matlabClassName + "_" + name;
	FileWriter file(toolboxPath + "/" + full_name + ".m", verbose, "%");

	string output;
	if(returnVal.isPair)
		output = "[ r1 r2 ] = ";
	else if(returnVal.category1 == ReturnValue::VOID)
		output = "";
	else
		output = "r = ";
	file.oss << "function " << output << full_name << "(varargin)\n";
	file.oss << "  " << output << wrapperName << "(" << id << ", varargin{:});\n";
	file.oss << "end\n";

	file.emit(true);
}

/* ************************************************************************* */
string StaticMethod::wrapper_fragment(FileWriter& file,
		const string& matlabClassName, const string& cppClassName,
		int id,	const vector<string>& using_namespaces) const {
	
	const string full_name = matlabClassName + "_" + name;
	const string wrapFunctionName = matlabClassName + "_" + name + "_" + boost::lexical_cast<string>(id);

	// call
	file.oss << "void " << wrapFunctionName << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
	// start
	file.oss << "{\n";
	generateUsingNamespace(file, using_namespaces);

  if(returnVal.isPair)
  {
      if(returnVal.category1 == ReturnValue::CLASS)
        file.oss << "  typedef boost::shared_ptr<"  << returnVal.qualifiedType1("::")  << "> Shared" <<  returnVal.type1 << ";"<< endl;
      if(returnVal.category2 == ReturnValue::CLASS)
        file.oss << "  typedef boost::shared_ptr<"  << returnVal.qualifiedType2("::")  << "> Shared" <<  returnVal.type2 << ";"<< endl;
  }
  else
      if(returnVal.category1 == ReturnValue::CLASS)
        file.oss << "  typedef boost::shared_ptr<"  << returnVal.qualifiedType1("::")  << "> Shared" <<  returnVal.type1 << ";"<< endl;

  file.oss << "  typedef boost::shared_ptr<"  << cppClassName  << "> Shared;" << endl;

  // check arguments
  // NOTE: for static functions, there is no object passed
  file.oss << "  checkArguments(\"" << full_name << "\",nargout,nargin," << args.size() << ");\n";

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(file,0); // We start at 0 because there is no self object

  file.oss << "  ";

  // call method with default type
  if (returnVal.type1!="void")
    file.oss << returnVal.return_type(true,ReturnValue::pair) << " result = ";
  file.oss << cppClassName  << "::" << name << "(" << args.names() << ");\n";

  // wrap result
  // example: out[0]=wrap<bool>(result);
  returnVal.wrap_result(file);

  // finish
  file.oss << "}\n";

	return wrapFunctionName;
}

/* ************************************************************************* */
