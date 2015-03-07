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
 * @author Richard Roberts
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

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
void Deconstructor::proxy_fragment(FileWriter& file,
		const std::string& wrapperName,
		const std::string& matlabUniqueName, int id) const {

	file.oss << "    function delete(obj)\n";
	file.oss << "      " << wrapperName << "(" << id << ", obj.ptr_" << matlabUniqueName << ");\n";
	file.oss << "    end\n";
}

/* ************************************************************************* */
string Deconstructor::wrapper_fragment(FileWriter& file,
				 const string& cppClassName,
				 const string& matlabUniqueName,
				 int id) const {
  
	const string matlabName = matlab_wrapper_name(matlabUniqueName);

	const string wrapFunctionName = matlabUniqueName + "_deconstructor_" + boost::lexical_cast<string>(id);
    
  file.oss << "void " << wrapFunctionName << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])" << endl;
  file.oss << "{" << endl;
  file.oss << "  typedef boost::shared_ptr<"  << cppClassName  << "> Shared;" << endl;
  //Deconstructor takes 1 arg, the mxArray obj
  file.oss << "  checkArguments(\"" << matlabName << "\",nargout,nargin," << "1" << ");" << endl;
	file.oss << "  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));\n";
	file.oss << "  Collector_" << matlabUniqueName << "::iterator item;\n";
	file.oss << "  item = collector_" << matlabUniqueName << ".find(self);\n";
	file.oss << "  if(item != collector_" << matlabUniqueName << ".end()) {\n";
  file.oss << "    delete self;\n";
	file.oss << "    collector_" << matlabUniqueName << ".erase(item);\n";
	file.oss << "  }\n";
  file.oss << "}" << endl;

	return wrapFunctionName;
}

/* ************************************************************************* */
