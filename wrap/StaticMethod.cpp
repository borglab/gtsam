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
 * @author Richard Roberts
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
void StaticMethod::addOverload(bool verbose, const std::string& name,
		const ArgumentList& args, const ReturnValue& retVal) {
	this->verbose = verbose;
	this->name = name;
	this->argLists.push_back(args);
	this->returnVals.push_back(retVal);
}

/* ************************************************************************* */
void StaticMethod::proxy_wrapper_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
																		 const string& cppClassName,
																		 const std::string& matlabQualName,
																		 const std::string& matlabUniqueName,
																		 const string& wrapperName,
																		 const TypeAttributesTable& typeAttributes,
																		 vector<string>& functionNames) const {

    string upperName = name;  upperName[0] = std::toupper(upperName[0], std::locale());

	proxyFile.oss << "    function varargout = " << upperName << "(varargin)\n";
	//Comments for documentation
	string up_name  = boost::to_upper_copy(name);
	proxyFile.oss << "      % " << up_name << " usage:";
	unsigned int argLCount = 0;
	BOOST_FOREACH(ArgumentList argList, argLists) 
    { 
        proxyFile.oss << " " << name << "(";
        unsigned int i = 0;
		BOOST_FOREACH(const Argument& arg, argList) 
		{
		    if(i != argList.size()-1)
		        proxyFile.oss << arg.type << " " << arg.name << ", ";
            else
		        proxyFile.oss << arg.type << " " << arg.name;
		    i++;
        }
        if(argLCount != argLists.size()-1)
            proxyFile.oss << "), ";
        else
            proxyFile.oss << ") : returns " << returnVals[0].return_type(false, returnVals[0].pair) << endl; 
        argLCount++;
    }

	proxyFile.oss << "      % " << "Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html" << endl;
	proxyFile.oss << "      % " << "" << endl;
	proxyFile.oss << "      % " << "Usage" << endl;
    BOOST_FOREACH(ArgumentList argList, argLists) 
    { 
        proxyFile.oss << "      % " << up_name << "(";
        unsigned int i = 0;
		BOOST_FOREACH(const Argument& arg, argList) 
		{
		    if(i != argList.size()-1)
		        proxyFile.oss << arg.type << " " << arg.name << ", ";
            else
		        proxyFile.oss << arg.type << " " << arg.name;
		    i++;
        }
        proxyFile.oss << ")" << endl;
    }


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
			proxyFile.oss << "isa(varargin{" << i+1 << "},'" << args[i].matlabClass(".") << "')";
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
		proxyFile.oss << "        " << output << wrapperName << "(" << id << ", varargin{:});\n";

		// Output C++ wrapper code
		
		const string wrapFunctionName = wrapper_fragment(
			wrapperFile, cppClassName, matlabUniqueName, overload, id, typeAttributes);

		// Add to function list
		functionNames.push_back(wrapFunctionName);

	}
	
  proxyFile.oss << "      else\n";
	proxyFile.oss << "        error('Arguments do not match any overload of function " <<
		matlabQualName << "." << upperName << "');" << endl;

	proxyFile.oss << "      end\n";
	proxyFile.oss << "    end\n";
}

/* ************************************************************************* */
string StaticMethod::wrapper_fragment(FileWriter& file, 
			    const string& cppClassName,
			    const string& matlabUniqueName,
					int overload,
					int id,
					const TypeAttributesTable& typeAttributes) const {

  // generate code

	const string wrapFunctionName = matlabUniqueName + "_" + name + "_" + boost::lexical_cast<string>(id);

	const ArgumentList& args = argLists[overload];
	const ReturnValue& returnVal = returnVals[overload];

	// call
	file.oss << "void " << wrapFunctionName << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
	// start
	file.oss << "{\n";

	returnVal.wrapTypeUnwrap(file);

  file.oss << "  typedef boost::shared_ptr<"  << cppClassName  << "> Shared;" << endl;

  // check arguments
  // NOTE: for static functions, there is no object passed
  file.oss << "  checkArguments(\"" << matlabUniqueName << "." << name << "\",nargout,nargin," << args.size() << ");\n";

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(file,0); // We start at 0 because there is no self object

  // call method with default type and wrap result
  if (returnVal.type1!="void")
		returnVal.wrap_result(cppClassName+"::"+name+"("+args.names()+")", file, typeAttributes);
	else
		file.oss << cppClassName+"::"+name+"("+args.names()+");\n";

  // finish
  file.oss << "}\n";

	return wrapFunctionName;
}

/* ************************************************************************* */
