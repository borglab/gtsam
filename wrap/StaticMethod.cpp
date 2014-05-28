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

#include "StaticMethod.h"
#include "utilities.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>

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
void StaticMethod::proxy_wrapper_fragments(FileWriter& file,
    FileWriter& wrapperFile, const string& cppClassName,
    const std::string& matlabQualName, const std::string& matlabUniqueName,
    const string& wrapperName, const TypeAttributesTable& typeAttributes,
    vector<string>& functionNames) const {

  string upperName = name;
  upperName[0] = std::toupper(upperName[0], std::locale());

  file.oss << "    function varargout = " << upperName << "(varargin)\n";
  //Comments for documentation
  string up_name = boost::to_upper_copy(name);
  file.oss << "      % " << up_name << " usage: ";
  unsigned int argLCount = 0;
  BOOST_FOREACH(ArgumentList argList, argLists) {
    argList.emit_prototype(file, name);
    if (argLCount != argLists.size() - 1)
      file.oss << ", ";
    else
      file.oss << " : returns "
          << returnVals[0].return_type(false, returnVals[0].pair) << endl;
    argLCount++;
  }

  file.oss << "      % "
      << "Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html"
      << endl;
  file.oss << "      % " << "" << endl;
  file.oss << "      % " << "Usage" << endl;
  BOOST_FOREACH(ArgumentList argList, argLists) {
    file.oss << "      % ";
    argList.emit_prototype(file, up_name);
    file.oss << endl;
  }

  // Check arguments for all overloads
  for (size_t overload = 0; overload < argLists.size(); ++overload) {

    // Output proxy matlab code
    file.oss << "      " << (overload == 0 ? "" : "else");
    const int id = (int) functionNames.size();
    argLists[overload].emit_conditional_call(file, returnVals[overload],
        wrapperName, id, true); // last bool is to indicate static !

    // Output C++ wrapper code
    const string wrapFunctionName = wrapper_fragment(wrapperFile, cppClassName,
        matlabUniqueName, (int) overload, id, typeAttributes);

    // Add to function list
    functionNames.push_back(wrapFunctionName);
  }
  file.oss << "      else\n";
  file.oss << "        error('Arguments do not match any overload of function "
      << matlabQualName << "." << upperName << "');" << endl;
  file.oss << "      end\n";

  file.oss << "    end\n";
}

/* ************************************************************************* */
string StaticMethod::wrapper_fragment(FileWriter& file,
    const string& cppClassName, const string& matlabUniqueName, int overload,
    int id, const TypeAttributesTable& typeAttributes) const {

  // generate code

  const string wrapFunctionName = matlabUniqueName + "_" + name + "_"
      + boost::lexical_cast<string>(id);

  const ArgumentList& args = argLists[overload];
  const ReturnValue& returnVal = returnVals[overload];

  // call
  file.oss << "void " << wrapFunctionName
      << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  // start
  file.oss << "{\n";

  returnVal.wrapTypeUnwrap(file);

  file.oss << "  typedef boost::shared_ptr<" << cppClassName << "> Shared;"
      << endl;

  // check arguments
  // NOTE: for static functions, there is no object passed
  file.oss << "  checkArguments(\"" << matlabUniqueName << "." << name
      << "\",nargout,nargin," << args.size() << ");\n";

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(file, 0); // We start at 0 because there is no self object

  // call method with default type and wrap result
  if (returnVal.type1 != "void")
    returnVal.wrap_result(cppClassName + "::" + name + "(" + args.names() + ")",
        file, typeAttributes);
  else
    file.oss << cppClassName + "::" + name + "(" + args.names() + ");\n";

  // finish
  file.oss << "}\n";

  return wrapFunctionName;
}

/* ************************************************************************* */
