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
void Method::addOverload(bool verbose, bool is_const, const std::string& name_,
    const ArgumentList& args, const ReturnValue& retVal,
    const Qualified& instName) {

  Function::addOverload(verbose, name_, args, retVal);
  is_const_ = is_const;
}

/* ************************************************************************* */
void Method::proxy_wrapper_fragments(FileWriter& proxyFile,
    FileWriter& wrapperFile, const string& cppClassName,
    const std::string& matlabQualName, const std::string& matlabUniqueName,
    const string& wrapperName, const TypeAttributesTable& typeAttributes,
    vector<string>& functionNames) const {

  // Create function header
  proxyFile.oss << "    function varargout = " << name_ << "(this, varargin)\n";

  // Emit comments for documentation
  string up_name = boost::to_upper_copy(name_);
  proxyFile.oss << "      % " << up_name << " usage: ";
  unsigned int argLCount = 0;
  BOOST_FOREACH(ArgumentList argList, argLists) {
    argList.emit_prototype(proxyFile, name_);
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
      argList.emit_prototype(proxyFile, name_);
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
        matlabUniqueName, 0, id, typeAttributes, templateArgValue_);

    // Add to function list
    functionNames.push_back(wrapFunctionName);
  } else {
    // Check arguments for all overloads
    for (size_t overload = 0; overload < argLists.size(); ++overload) {

      // Output proxy matlab code
      proxyFile.oss << "      " << (overload == 0 ? "" : "else");
      const int id = (int) functionNames.size();
      string expanded = wrapperName;
      if (!templateArgValue_.empty())
        expanded += templateArgValue_.name;
      argLists[overload].emit_conditional_call(proxyFile, returnVals[overload],
          expanded, id);

      // Output C++ wrapper code
      const string wrapFunctionName = wrapper_fragment(wrapperFile,
          cppClassName, matlabUniqueName, overload, id, typeAttributes,
          templateArgValue_);

      // Add to function list
      functionNames.push_back(wrapFunctionName);
    }
    proxyFile.oss << "      else\n";
    proxyFile.oss
        << "        error('Arguments do not match any overload of function "
        << matlabQualName << "." << name_ << "');" << endl;
    proxyFile.oss << "      end\n";
  }

  proxyFile.oss << "    end\n";
}

/* ************************************************************************* */
string Method::wrapper_fragment(FileWriter& wrapperFile,
    const string& cppClassName, const string& matlabUniqueName, int overload,
    int id, const TypeAttributesTable& typeAttributes,
    const Qualified& instName) const {

  // generate code

  const string wrapFunctionName = matlabUniqueName + "_" + name_ + "_"
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
  wrapperFile.oss << "  checkArguments(\"" << name_ << "\",nargout,nargin-1,"
      << args.size() << ");\n";

  // get class pointer
  // example: shared_ptr<Test> = unwrap_shared_ptr< Test >(in[0], "Test");
  wrapperFile.oss << "  Shared obj = unwrap_shared_ptr<" << cppClassName
      << ">(in[0], \"ptr_" << matlabUniqueName << "\");" << endl;
  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(wrapperFile, 1);

  // call method and wrap result
  // example: out[0]=wrap<bool>(self->return_field(t));
  string expanded = "obj->" + name_;
  if (!instName.empty())
    expanded += ("<" + instName.qualifiedName("::") + ">");
  expanded += ("(" + args.names() + ")");
  if (returnVal.type1.name != "void")
    returnVal.wrap_result(expanded, wrapperFile, typeAttributes);
  else
    wrapperFile.oss << "  " + expanded + ";\n";

  // finish
  wrapperFile.oss << "}\n";

  return wrapFunctionName;
}

/* ************************************************************************* */
