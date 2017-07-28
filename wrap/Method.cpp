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
#include "Class.h"
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
                         boost::optional<const Qualified> instName,
                         bool verbose) {
  bool first = MethodBase::addOverload(name, args, retVal, instName, verbose);
  if (first)
    is_const_ = is_const;
  else if (is_const && !is_const_)
    throw std::runtime_error(
        "Method::addOverload now designated as const whereas before it was "
        "not");
  else if (!is_const && is_const_)
    throw std::runtime_error(
        "Method::addOverload now designated as non-const whereas before it "
        "was");
  return first;
}

/* ************************************************************************* */
void Method::proxy_header(FileWriter& proxyFile) const {
  proxyFile.oss << "    function varargout = " << matlabName()
                << "(this, varargin)\n";
}

/* ************************************************************************* */
string Method::wrapper_call(FileWriter& wrapperFile, Str cppClassName,
                            Str matlabUniqueName,
                            const ArgumentList& args) const {
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
void Method::emit_cython_pxd(FileWriter& file, const Class& cls) const {
  for (size_t i = 0; i < nrOverloads(); ++i) {
    file.oss << "        ";
    returnVals_[i].emit_cython_pxd(file, cls.pxdClassName(), cls.templateArgs);
    file.oss << pyRename(name_) + " \"" + name_ + "\""
             << "(";
    argumentList(i).emit_cython_pxd(file, cls.pxdClassName(), cls.templateArgs);
    file.oss << ")";
    // if (is_const_) file.oss << " const";
    file.oss << " except +";
    file.oss << "\n";
  }
}

/* ************************************************************************* */
void Method::emit_cython_pyx_no_overload(FileWriter& file,
                                         const Class& cls) const {
  string funcName = pyRename(name_);
  // leverage python's special treatment for print
  if (funcName == "print_") {
    //file.oss << "    def __str__(self):\n        self.print_('')\n        return ''\n";
    file.oss << "    def __str__(self):\n";
    file.oss << "        strBuf = RedirectCout()\n";
    file.oss << "        self.print_('')\n";
    file.oss << "        return strBuf.str()\n";
  }
  // Function definition
  file.oss << "    def " << funcName;
  // modify name of function instantiation as python doesn't allow overloads
  // e.g. template<T={A,B,C}> funcName(...) --> funcNameA, funcNameB, funcNameC
  if (templateArgValue_) file.oss << templateArgValue_->pyxClassName();
  // funtion arguments
  file.oss << "(self";
  if (argumentList(0).size() > 0) file.oss << ", ";
  argumentList(0).emit_cython_pyx(file);
  file.oss << "):\n";

  /// Call cython corresponding function and return
  file.oss << argumentList(0).pyx_convertEigenTypeAndStorageOrder("        ");
  string caller = "self." + cls.shared_pxd_obj_in_pyx() + ".get()";
  string ret = pyx_functionCall(caller, funcName, 0);
  if (!returnVals_[0].isVoid()) {
    file.oss << "        cdef " << returnVals_[0].pyx_returnType()
             << " ret = " << ret << "\n";
    file.oss << "        return " << returnVals_[0].pyx_casting("ret") << "\n";
  } else {
    file.oss << "        " << ret << "\n";
  }
}

/* ************************************************************************* */
void Method::emit_cython_pyx(FileWriter& file, const Class& cls) const {
  string funcName = pyRename(name_);
  // For template function: modify name of function instantiation as python
  // doesn't allow overloads
  // e.g. template<T={A,B,C}> funcName(...) --> funcNameA, funcNameB, funcNameC
  string instantiatedName =
      (templateArgValue_) ? funcName + templateArgValue_->pyxClassName() : funcName;

  size_t N = nrOverloads();
  // It's easy if there's no overload
  if (N == 1) {
    emit_cython_pyx_no_overload(file, cls);
    return;
  }

  // Dealing with overloads..
  file.oss << "    def " << instantiatedName << "(self, *args, **kwargs):\n";
  for (size_t i = 0; i < N; ++i) {
    file.oss << "        success, results = self." << instantiatedName << "_" << i
             << "(*args, **kwargs)\n";
    file.oss << "        if success:\n            return results\n";
  }
  file.oss << "        raise TypeError('Could not find the correct overload')\n";

  for (size_t i = 0; i < N; ++i) {
    ArgumentList args = argumentList(i);
    file.oss << "    def " + instantiatedName + "_" + to_string(i) +
                    "(self, *args, **kwargs):\n";
    file.oss << pyx_resolveOverloadParams(args, false); // lazy: always return None even if it's a void function

    /// Call cython corresponding function
    file.oss << argumentList(i).pyx_convertEigenTypeAndStorageOrder("        ");
    string caller = "self." + cls.shared_pxd_obj_in_pyx() + ".get()";

    string ret = pyx_functionCall(caller, funcName, i);
    if (!returnVals_[i].isVoid()) {
      file.oss << "        cdef " << returnVals_[i].pyx_returnType()
              << " ret = " << ret << "\n";
      file.oss << "        return True, " << returnVals_[i].pyx_casting("ret") << "\n";
    } else {
      file.oss << "        " << ret << "\n";
      file.oss << "        return True, None\n";
    }
  }
}
/* ************************************************************************* */
