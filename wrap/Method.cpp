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
 * @author Richard Roberts
 **/

#include "Method.h"
#include "utilities.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Method::addOverload(bool verbose, bool is_const, const std::string& name,
    const ArgumentList& args, const ReturnValue& retVal) {
  if (name.empty())
    this->name = name;
  else if (this->name != name)
    throw std::runtime_error(
        "Method::addOverload: tried to add overload with name " + name
            + " instead of expected " + this->name);
  this->verbose_ = verbose;
  this->is_const_ = is_const;
  this->name = name;
  this->argLists.push_back(args);
  this->returnVals.push_back(retVal);
}

/* ************************************************************************* */
void Method::proxy_wrapper_fragments(FileWriter& proxyFile,
    FileWriter& wrapperFile, const string& cppClassName,
    const std::string& matlabQualName, const std::string& matlabUniqueName,
    const string& wrapperName, const TypeAttributesTable& typeAttributes,
    vector<string>& functionNames) const {

  // Create function header
  proxyFile.oss << "    function varargout = " << name << "(this, varargin)\n";

  // Emit comments for documentation
  string up_name = boost::to_upper_copy(name);
  proxyFile.oss << "      % " << up_name << " usage: ";
  unsigned int argLCount = 0;
  BOOST_FOREACH(ArgumentList argList, argLists) {
    argList.emit_prototype(proxyFile, name);
    if (argLCount != argLists.size() - 1)
      proxyFile.oss << ", ";
    else
      proxyFile.oss << " : returns " << returnVals[0].return_type(false)
          << endl;
    argLCount++;
  }

  // Emit URL to Doxygen page
  proxyFile.oss << "      % "
      << "Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html"
      << endl;

  // Document all overloads, if any
  if (argLists.size() > 1) {
    proxyFile.oss << "      % " << "" << endl;
    proxyFile.oss << "      % " << "Method Overloads" << endl;
    BOOST_FOREACH(ArgumentList argList, argLists) {
      proxyFile.oss << "      % ";
      argList.emit_prototype(proxyFile, name);
      proxyFile.oss << endl;
    }
  }

  // Handle special case of single overload with all numeric arguments
  if (argLists.size() == 1 && argLists[0].allScalar()) {
    // Output proxy matlab code
    proxyFile.oss << "      ";
    const int id = (int) functionNames.size();
    argLists[0].emit_call(proxyFile, returnVals[0], wrapperName, id);

    // Output C++ wrapper code
    const string wrapFunctionName = wrapper_fragment(wrapperFile, cppClassName,
        matlabUniqueName, 0, id, typeAttributes);

    // Add to function list
    functionNames.push_back(wrapFunctionName);
  } else {
    // Check arguments for all overloads
    for (size_t overload = 0; overload < argLists.size(); ++overload) {

      // Output proxy matlab code
      proxyFile.oss << "      " << (overload == 0 ? "" : "else");
      const int id = (int) functionNames.size();
      argLists[overload].emit_conditional_call(proxyFile, returnVals[overload],
          wrapperName, id);

      // Output C++ wrapper code
      const string wrapFunctionName = wrapper_fragment(wrapperFile,
          cppClassName, matlabUniqueName, overload, id, typeAttributes);

      // Add to function list
      functionNames.push_back(wrapFunctionName);
    }
    proxyFile.oss << "      else\n";
    proxyFile.oss
        << "        error('Arguments do not match any overload of function "
        << matlabQualName << "." << name << "');" << endl;
    proxyFile.oss << "      end\n";
  }

  proxyFile.oss << "    end\n";
}

/* ************************************************************************* */
string Method::wrapper_fragment(FileWriter& wrapperFile,
    const string& cppClassName, const string& matlabUniqueName, int overload,
    int id, const TypeAttributesTable& typeAttributes) const {

  // generate code

  const string wrapFunctionName = matlabUniqueName + "_" + name + "_"
      + boost::lexical_cast<string>(id);

  const ArgumentList& args = argLists[overload];
  const ReturnValue& returnVal = returnVals[overload];

  // call
  wrapperFile.oss << "void " << wrapFunctionName
      << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  // start
  wrapperFile.oss << "{\n";

  returnVal.wrapTypeUnwrap(wrapperFile);

  wrapperFile.oss << "  typedef boost::shared_ptr<" << cppClassName
      << "> Shared;" << endl;

  // check arguments
  // extra argument obj -> nargin-1 is passed !
  // example: checkArguments("equals",nargout,nargin-1,2);
  wrapperFile.oss << "  checkArguments(\"" << name << "\",nargout,nargin-1,"
      << args.size() << ");\n";

  // get class pointer
  // example: shared_ptr<Test> = unwrap_shared_ptr< Test >(in[0], "Test");
  wrapperFile.oss << "  Shared obj = unwrap_shared_ptr<" << cppClassName
      << ">(in[0], \"ptr_" << matlabUniqueName << "\");" << endl;
  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(wrapperFile, 1);

  // call method and wrap result
  // example: out[0]=wrap<bool>(self->return_field(t));
  if (returnVal.type1.name != "void")
    returnVal.wrap_result("obj->" + name + "(" + args.names() + ")",
        wrapperFile, typeAttributes);
  else
    wrapperFile.oss << "  obj->" + name + "(" + args.names() + ");\n";

  // finish
  wrapperFile.oss << "}\n";

  return wrapFunctionName;
}

/* ************************************************************************* */
