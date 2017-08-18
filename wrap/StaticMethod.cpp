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
  for(size_t i = 0; i < nrOverloads(); ++i) {
    file.oss << "        @staticmethod\n";
    file.oss << "        ";
    returnVals_[i].emit_cython_pxd(file, cls.pxdClassName(), cls.templateArgs);
    file.oss << name_ + ((i>0)?"_" + to_string(i):"") << " \"" << name_ << "\"" << "(";
    argumentList(i).emit_cython_pxd(file, cls.pxdClassName(), cls.templateArgs);
    file.oss << ") except +\n";
  }
}

/* ************************************************************************* */
void StaticMethod::emit_cython_wrapper_pxd(FileWriter& file,
    const Class& cls) const {
  if (nrOverloads() > 1) {
    for (size_t i = 0; i < nrOverloads(); ++i) {
      string funcName = name_ + "_" + to_string(i);
      file.oss << "    @staticmethod\n";
      file.oss << "    cdef tuple " + funcName + "(tuple args, dict kwargs)\n";
    }
  }
}

/* ************************************************************************* */
void StaticMethod::emit_cython_pyx_no_overload(FileWriter& file,
                                               const Class& cls) const {
  assert(nrOverloads() == 1);
  file.oss << "    @staticmethod\n";
  file.oss << "    def " << name_ << "(";
  argumentList(0).emit_cython_pyx(file);
  file.oss << "):\n";

  /// Call cython corresponding function and return
  file.oss << argumentList(0).pyx_convertEigenTypeAndStorageOrder("        ");
  string call = pyx_functionCall(cls.pxd_class_in_pyx(), name_, 0);
  file.oss << "        ";
  if (!returnVals_[0].isVoid()) {
    file.oss << "return " << returnVals_[0].pyx_casting(call) << "\n";
  } else
    file.oss << call << "\n";
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
  file.oss << "    @staticmethod # overloaded\n";
  file.oss << "    def " << name_ << "(*args, **kwargs):\n";
  for (size_t i = 0; i < N; ++i) {
    string funcName = name_ + "_" + to_string(i);
    file.oss << "        success, results = " << cls.pyxClassName() << "."
             << funcName << "(args, kwargs)\n";
    file.oss << "        if success:\n            return results\n";
  }
  file.oss << "        raise TypeError('Could not find the correct overload')\n\n";

  // Create cdef methods for all overloaded methods
  for(size_t i = 0; i < N; ++i) {
    string funcName = name_ + "_" + to_string(i);
    file.oss << "    @staticmethod\n";
    file.oss << "    cdef tuple " + funcName + "(tuple args, dict kwargs):\n";
    file.oss << "        cdef list __params\n";
    if (!returnVals_[i].isVoid()) {
      file.oss << "        cdef " << returnVals_[i].pyx_returnType() << " return_value\n";
    }
    file.oss << "        try:\n";
    ArgumentList args = argumentList(i);
    file.oss << pyx_resolveOverloadParams(args, false, 3);

    /// Call cython corresponding function and return
    file.oss << args.pyx_convertEigenTypeAndStorageOrder("            ");
    string pxdFuncName = name_ + ((i>0)?"_" + to_string(i):"");
    string call = pyx_functionCall(cls.pxd_class_in_pyx(), pxdFuncName, i);
    if (!returnVals_[i].isVoid()) {
      file.oss << "            return_value = " << call << "\n";
      file.oss << "            return True, " << returnVals_[i].pyx_casting("return_value") << "\n";
    } else {
      file.oss << "            " << call << "\n";
      file.oss << "            return True, None\n";
    }
    file.oss << "        except:\n";
    file.oss << "            return False, None\n\n";
  }
}

/* ************************************************************************* */
