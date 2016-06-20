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

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
bool Method::addOverload(Str name, const ArgumentList& args,
    const ReturnValue& retVal, bool is_const,
    boost::optional<const Qualified> instName, bool verbose) {
  bool first = MethodBase::addOverload(name, args, retVal, instName, verbose);
  if (first)
    is_const_ = is_const;
  else if (is_const && !is_const_)
    throw std::runtime_error(
        "Method::addOverload now designated as const whereas before it was not");
  else if (!is_const && is_const_)
    throw std::runtime_error(
        "Method::addOverload now designated as non-const whereas before it was");
  return first;
}

/* ************************************************************************* */
void Method::proxy_header(FileWriter& proxyFile) const {
  proxyFile.oss << "    function varargout = " << matlabName()
      << "(this, varargin)\n";
}

/* ************************************************************************* */
string Method::wrapper_call(FileWriter& wrapperFile, Str cppClassName,
    Str matlabUniqueName, const ArgumentList& args) const {
  // check arguments
  // extra argument obj -> nargin-1 is passed !
  // example: checkArguments("equals",nargout,nargin-1,2);
  wrapperFile.oss << "  checkArguments(\"" << matlabName()
      << "\",nargout,nargin-1," << args.size() << ");\n";

  // get class pointer
  // example: shared_ptr<Test> = unwrap_shared_ptr< Test >(in[0], "Test");
  wrapperFile.oss << "  Shared obj = unwrap_shared_ptr<" << cppClassName
      << ">(in[0], \"ptr_" << matlabUniqueName << "\");" << endl;

  // unwrap arguments, see Argument.cpp, we start at 1 as first is obj
  args.matlab_unwrap(wrapperFile, 1);

  // call method and wrap result
  // example: out[0]=wrap<bool>(obj->return_field(t));
  string expanded = "obj->" + name_;
  if (templateArgValue_)
    expanded += ("<" + templateArgValue_->qualifiedName("::") + ">");

  return expanded;
}

/* ************************************************************************* */
