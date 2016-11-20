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
#include "Class.h"

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
string StaticMethod::wrapper_call(FileWriter& wrapperFile, Str cppClassName,
    Str matlabUniqueName, const ArgumentList& args) const {
  // check arguments
  // NOTE: for static functions, there is no object passed
  wrapperFile.oss << "  checkArguments(\"" << matlabUniqueName << "." << name_
      << "\",nargout,nargin," << args.size() << ");\n";

  // unwrap arguments, see Argument.cpp
  args.matlab_unwrap(wrapperFile, 0); // We start at 0 because there is no self object

  // call method and wrap result
  // example: out[0]=wrap<bool>(staticMethod(t));
  string expanded = cppClassName + "::" + name_;
  if (templateArgValue_)
    expanded += ("<" + templateArgValue_->qualifiedName("::") + ">");

  return expanded;
}

/* ************************************************************************* */
void StaticMethod::emit_cython_pxd(FileWriter& file, const Class& cls) const {
  // don't support overloads for static method :-(
  for(size_t i = 0; i < nrOverloads(); ++i) {
    file.oss << "\t\t@staticmethod\n";
    file.oss << "\t\t";
    returnVals_[i].emit_cython_pxd(file, cls.cythonClass());
    file.oss << name_ + ((i>0)?"_" + to_string(i):"") << " \"" << name_ << "\"" << "(";
    argumentList(i).emit_cython_pxd(file, cls.cythonClass());
    file.oss << ")\n";
  }
}

/* ************************************************************************* */
void StaticMethod::emit_cython_pyx_no_overload(FileWriter& file,
                                               const Class& cls) const {
  assert(nrOverloads() == 1);
  file.oss << "\t@staticmethod\n";
  file.oss << "\tdef " << name_ << "(";
  argumentList(0).emit_cython_pyx(file);
  file.oss << "):\n";

  /// Call cython corresponding function and return
  string ret = pyx_functionCall(cls.pyxCythonClass(), name_, 0);
  file.oss << "\t\t";
  if (!returnVals_[0].isVoid()) {
    file.oss << "return " << returnVals_[0].pyx_casting(ret) << "\n";
  } else
    file.oss << ret << "\n";
  file.oss << "\n";
}

/* ************************************************************************* */
void StaticMethod::emit_cython_pyx(FileWriter& file, const Class& cls) const {
  size_t N = nrOverloads();
  if (N == 1) {
    emit_cython_pyx_no_overload(file, cls);
    return;
  }

  // Dealing with overloads..
  file.oss << "\t@staticmethod\n";
  file.oss << "\tdef " << name_ << "(*args, **kwargs):\n";
  file.oss << pyx_checkDuplicateNargsKwArgs();
  for (size_t i = 0; i < N; ++i) {
    string funcName = name_ + "_" + to_string(i);
    file.oss << "\t\tsuccess, results = " << cls.pythonClass() << "."
             << funcName << "(*args, **kwargs)\n";
    file.oss << "\t\tif success:\n\t\t\treturn results\n";
  }
  file.oss << "\t\traise TypeError('Could not find the correct overload')\n";

  for(size_t i = 0; i < N; ++i) {
    file.oss << "\t@staticmethod\n";

    string funcName = name_ + "_" + to_string(i);
    string pxdFuncName = name_ + ((i>0)?"_" + to_string(i):"");
    ArgumentList args = argumentList(i);
    file.oss << "\tdef " + funcName + "(*args, **kwargs):\n";
    file.oss << pyx_resolveOverloadParams(args);

    /// Call cython corresponding function and return
    string ret = pyx_functionCall(cls.pyxCythonClass(), pxdFuncName, i);
    if (!returnVals_[i].isVoid()) {
      file.oss << "\t\tcdef " << returnVals_[i].pyx_returnType()
              << " ret = " << ret << "\n";
      file.oss << "\t\treturn True, " << returnVals_[i].pyx_casting("ret") << "\n";
    }
    else {
      file.oss << "\t\t" << ret << "\n";
      file.oss << "\t\treturn True, None\n";
    } 
    file.oss << "\n";
  }
}

/* ************************************************************************* */
