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
  this->verbose_ = verbose;
  this->is_const_ = is_const;
  this->name = name;
  this->argLists.push_back(args);
  this->returnVals.push_back(retVal);
}

/* ************************************************************************* */
void Method::proxy_wrapper_fragments(FileWriter& file, FileWriter& wrapperFile,
    const string& cppClassName, const std::string& matlabQualName,
    const std::string& matlabUniqueName, const string& wrapperName,
    const TypeAttributesTable& typeAttributes,
    vector<string>& functionNames) const {

  // Create function header
  file.oss << "    function varargout = " << name << "(this, varargin)\n";

  // Emit comments for documentation
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

  // Emit URL to Doxygen page
  file.oss << "      % "
      << "Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html"
      << endl;

  // Document all overloads, if any
  if (argLists.size() > 1) {
    file.oss << "      % " << "" << endl;
    file.oss << "      % " << "Method Overloads" << endl;
    BOOST_FOREACH(ArgumentList argList, argLists) {
      file.oss << "      % ";
      argList.emit_prototype(file, name);
      file.oss << endl;
    }
  }

  // Handle special case of single overload with all numeric arguments
  if (argLists.size() == 1 && argLists[0].allScalar()) {
    // Output proxy matlab code
    file.oss << "      ";
    const int id = (int) functionNames.size();
    argLists[0].emit_call(file, returnVals[0], wrapperName, id);

    // Output C++ wrapper code
    const string wrapFunctionName = wrapper_fragment(wrapperFile, cppClassName,
        matlabUniqueName, 0, id, typeAttributes);

    // Add to function list
    functionNames.push_back(wrapFunctionName);
  } else {
    // Check arguments for all overloads
    for (size_t overload = 0; overload < argLists.size(); ++overload) {

      // Output proxy matlab code
      file.oss << "      " << (overload == 0 ? "" : "else");
      const int id = (int) functionNames.size();
      argLists[overload].emit_conditional_call(file, returnVals[overload],
          wrapperName, id);

      // Output C++ wrapper code
      const string wrapFunctionName = wrapper_fragment(wrapperFile,
          cppClassName, matlabUniqueName, overload, id, typeAttributes);

      // Add to function list
      functionNames.push_back(wrapFunctionName);
    }
    file.oss << "      else\n";
    file.oss
        << "        error('Arguments do not match any overload of function "
        << matlabQualName << "." << name << "');" << endl;
    file.oss << "      end\n";
  }

  file.oss << "    end\n";
}

/* ************************************************************************* */
string Method::wrapper_fragment(FileWriter& file, const string& cppClassName,
    const string& matlabUniqueName, int overload, int id,
    const TypeAttributesTable& typeAttributes) const {

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

  if (returnVal.isPair) {
    if (returnVal.category1 == ReturnValue::CLASS)
      file.oss << "  typedef boost::shared_ptr<"
          << returnVal.qualifiedType1("::") << "> Shared" << returnVal.type1
          << ";" << endl;
    if (returnVal.category2 == ReturnValue::CLASS)
      file.oss << "  typedef boost::shared_ptr<"
          << returnVal.qualifiedType2("::") << "> Shared" << returnVal.type2
          << ";" << endl;
  } else if (returnVal.category1 == ReturnValue::CLASS)
    file.oss << "  typedef boost::shared_ptr<" << returnVal.qualifiedType1("::")
        << "> Shared" << returnVal.type1 << ";" << endl;

  file.oss << "  typedef boost::shared_ptr<" << cppClassName << "> Shared;"
      << endl;

  // check arguments
  // extra argument obj -> nargin-1 is passed !
  // example: checkArguments("equals",nargout,nargin-1,2);
  file.oss << "  checkArguments(\"" << name << "\",nargout,nargin-1,"
      << args.size() << ");\n";

  // get class pointer
  // example: shared_ptr<Test> = unwrap_shared_ptr< Test >(in[0], "Test");
  file.oss << "  Shared obj = unwrap_shared_ptr<" << cppClassName
      << ">(in[0], \"ptr_" << matlabUniqueName << "\");" << endl;
  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(file, 1);

  // call method and wrap result
  // example: out[0]=wrap<bool>(self->return_field(t));
  if (returnVal.type1 != "void")
    returnVal.wrap_result("obj->" + name + "(" + args.names() + ")", file,
        typeAttributes);
  else
    file.oss << "  obj->" + name + "(" + args.names() + ");\n";

  // finish
  file.oss << "}\n";

  return wrapFunctionName;
}

/* ************************************************************************* */
