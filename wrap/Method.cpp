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
#include <boost/lexical_cast.hpp>

#include "Method.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Method::proxy_fragment(FileWriter& file, const std::string& wrapperName, const int id) const {

	string output;
	if(returnVal.isPair)
		output = "[ r1 r2 ] = ";
	else if(returnVal.category1 == ReturnValue::VOID)
		output = "";
	else
		output = "r = ";
	file.oss << "    function " << output << name << "(varargin)\n";
	file.oss << "      " << output << wrapperName << "(" << id << ", varargin{:});\n";
	file.oss << "    end\n";
}

/* ************************************************************************* */
string Method::wrapper_fragment(FileWriter& file, 
			    const string& cppClassName,
			    const string& matlabClassName,
					int id,
			    const vector<string>& using_namespaces) const {

  // generate code

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
  // extra argument obj -> nargin-1 is passed !
  // example: checkArguments("equals",nargout,nargin-1,2);
  file.oss << "  checkArguments(\"" << name << "\",nargout,nargin-1," << args.size() << ");\n";

  // get class pointer
  // example: shared_ptr<Test> = unwrap_shared_ptr< Test >(in[0], "Test");
  file.oss << "  Shared obj = unwrap_shared_ptr<" << cppClassName << ">(in[0], \"" << cppClassName << "\");" << endl;
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

	return wrapFunctionName;
}

/* ************************************************************************* */
