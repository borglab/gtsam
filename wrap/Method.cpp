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
void Method::addOverload(bool verbose, bool is_const, const std::string& name,
		const ArgumentList& args, const ReturnValue& retVal) {
	this->verbose_ = verbose;
	this->is_const_ = is_const;
	this->name = name;
	this->argLists.push_back(args);
	this->returnVals.push_back(retVal);
}

void Method::proxy_wrapper_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
																		 const string& cppClassName,
																		 const string& matlabClassName,
																		 const string& wrapperName,
																		 const vector<string>& using_namespaces,
																		 const ReturnValue::TypeAttributesTable& typeAttributes,
																		 vector<string>& functionNames) const {

	proxyFile.oss << "    function varargout = " << name << "(this, varargin)\n";

	for(size_t overload = 0; overload < argLists.size(); ++overload) {
		const ArgumentList& args = argLists[overload];
		const ReturnValue& returnVal = returnVals[overload];
		size_t nrArgs = args.size();

		const int id = functionNames.size();

		// Output proxy matlab code

		// check for number of arguments...
		proxyFile.oss << "      " << (overload==0?"":"else") << "if length(varargin) == " << nrArgs;
		if (nrArgs>0) proxyFile.oss << " && ";
		// ...and their types
		bool first = true;
		for(size_t i=0;i<nrArgs;i++) {
			if (!first) proxyFile.oss << " && ";
			proxyFile.oss << "isa(varargin{" << i+1 << "},'" << args[i].matlabClass() << "')";
			first=false;
		}
		proxyFile.oss << "\n";

		// output call to C++ wrapper
		string output;
		if(returnVal.isPair)
			output = "[ varargout{1} varargout{2} ] = ";
		else if(returnVal.category1 == ReturnValue::VOID)
			output = "";
		else
			output = "varargout{1} = ";
		proxyFile.oss << "        " << output << wrapperName << "(" << id << ", this, varargin{:});\n";

		// Output C++ wrapper code
		
		const string wrapFunctionName = wrapper_fragment(
			wrapperFile, cppClassName, matlabClassName, overload, id, using_namespaces, typeAttributes);

		// Add to function list
		functionNames.push_back(wrapFunctionName);

	}
	
  proxyFile.oss << "      else\n";
	proxyFile.oss << "        error('Arguments do not match any overload of function " <<
		matlabClassName << "." << name << "');" << endl;

	proxyFile.oss << "      end\n";
	proxyFile.oss << "    end\n";
}

/* ************************************************************************* */
string Method::wrapper_fragment(FileWriter& file, 
			    const string& cppClassName,
			    const string& matlabClassName,
					int overload,
					int id,
			    const vector<string>& using_namespaces,
					const ReturnValue::TypeAttributesTable& typeAttributes) const {

  // generate code

	const string wrapFunctionName = matlabClassName + "_" + name + "_" + boost::lexical_cast<string>(id);

	const ArgumentList& args = argLists[overload];
	const ReturnValue& returnVal = returnVals[overload];

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
  file.oss << "  Shared obj = unwrap_shared_ptr<" << cppClassName << ">(in[0], \"ptr_" << matlabClassName << "\");" << endl;
  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(file,1);

  // call method and wrap result
  // example: out[0]=wrap<bool>(self->return_field(t));
  if (returnVal.type1!="void")
		returnVal.wrap_result("obj->"+name+"("+args.names()+")", file, typeAttributes);
	else
		file.oss << "  obj->"+name+"("+args.names()+");\n";

  // finish
  file.oss << "}\n";

	return wrapFunctionName;
}

/* ************************************************************************* */
