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
void StaticMethod::proxy_header(FileWriter& proxyFile) const {
  string upperName = matlabName();
  upperName[0] = toupper(upperName[0], locale());
  proxyFile.oss << "    function varargout = " << upperName << "(varargin)\n";
}

/* ************************************************************************* */
void StaticMethod::proxy_wrapper_fragments(FileWriter& proxyFile,
    FileWriter& wrapperFile, Str cppClassName, Str matlabQualName,
    Str matlabUniqueName, Str wrapperName,
    const TypeAttributesTable& typeAttributes,
    vector<string>& functionNames) const {

  // emit header, e.g., function varargout = templatedMethod(this, varargin)
  proxy_header(proxyFile);

  // Emit comments for documentation
  string up_name = boost::to_upper_copy(matlabName());
  proxyFile.oss << "      % " << up_name << " usage: ";
  usage_fragment(proxyFile, matlabName());

  // Emit URL to Doxygen page
  proxyFile.oss << "      % "
      << "Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html"
      << endl;

  // Handle special case of single overload with all numeric arguments
  if (nrOverloads() == 1 && argumentList(0).allScalar()) {
    // Output proxy matlab code
    // TODO: document why is it OK to not check arguments in this case
    proxyFile.oss << "      ";
    const int id = (int) functionNames.size();
    argumentList(0).emit_call(proxyFile, returnValue(0), wrapperName, id,
        isStatic());

    // Output C++ wrapper code
    const string wrapFunctionName = wrapper_fragment(wrapperFile, cppClassName,
        matlabUniqueName, 0, id, typeAttributes, templateArgValue_);

    // Add to function list
    functionNames.push_back(wrapFunctionName);
  } else {
    // Check arguments for all overloads
    for (size_t i = 0; i < nrOverloads(); ++i) {

      // Output proxy matlab code
      proxyFile.oss << "      " << (i == 0 ? "" : "else");
      const int id = (int) functionNames.size();
      argumentList(i).emit_conditional_call(proxyFile, returnValue(i),
          wrapperName, id, isStatic());

      // Output C++ wrapper code
      const string wrapFunctionName = wrapper_fragment(wrapperFile,
          cppClassName, matlabUniqueName, i, id, typeAttributes,
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
string StaticMethod::wrapper_fragment(FileWriter& wrapperFile, Str cppClassName,
    Str matlabUniqueName, int overload, int id,
    const TypeAttributesTable& typeAttributes,
    const Qualified& instName) const {

  // generate code

  const string wrapFunctionName = matlabUniqueName + "_" + name_ + "_"
      + boost::lexical_cast<string>(id);

  const ArgumentList& args = argumentList(overload);
  const ReturnValue& returnVal = returnValue(overload);

  // call
  wrapperFile.oss << "void " << wrapFunctionName
      << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  // start
  wrapperFile.oss << "{\n";

  returnVal.wrapTypeUnwrap(wrapperFile);

  wrapperFile.oss << "  typedef boost::shared_ptr<" << cppClassName
      << "> Shared;" << endl;

  // get call
  // for static methods: cppClassName::staticMethod<TemplateVal>
  // for instance methods: obj->instanceMethod<TemplateVal>
  string expanded = wrapper_call(wrapperFile, cppClassName, matlabUniqueName,
      args, returnVal, typeAttributes, instName);

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
string StaticMethod::wrapper_call(FileWriter& wrapperFile, Str cppClassName,
    Str matlabUniqueName, const ArgumentList& args,
    const ReturnValue& returnVal, const TypeAttributesTable& typeAttributes,
    const Qualified& instName) const {
  // check arguments
  // NOTE: for static functions, there is no object passed
  wrapperFile.oss << "  checkArguments(\"" << matlabUniqueName << "." << name_
      << "\",nargout,nargin," << args.size() << ");\n";

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(wrapperFile, 0); // We start at 0 because there is no self object

  // call method and wrap result
  // example: out[0]=wrap<bool>(staticMethod(t));
  string expanded = cppClassName + "::" + name_;
  if (!instName.empty())
    expanded += ("<" + instName.qualifiedName("::") + ">");

  return expanded;
}

/* ************************************************************************* */
void StaticMethod::python_wrapper(FileWriter& wrapperFile,
    Str className) const {
  wrapperFile.oss << "  .def(\"" << name_ << "\", &" << className << "::" << name_
      << ");\n";
}

/* ************************************************************************* */
