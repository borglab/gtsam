/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Method.h
 * @brief describes and generates code for methods
 * @author Frank Dellaert
 * @author Richard Roberts
 **/

#pragma once

#include "Function.h"

namespace wrap {

/// Method class
struct Method : public Function {

  /// Constructor creates empty object
  Method(bool verbose = true) :
      Function(verbose), is_const_(false) {
  }

  bool is_const_;

  // The first time this function is called, it initializes the class members
  // with those in rhs, but in subsequent calls it adds additional argument
  // lists as function overloads.
  void addOverload(bool verbose, bool is_const, const std::string& name,
      const ArgumentList& args, const ReturnValue& retVal,
      const Qualified& instName = Qualified());

  // MATLAB code generation
  // classPath is class directory, e.g., ../matlab/@Point2
  void proxy_wrapper_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
      const std::string& cppClassName, const std::string& matlabQualName,
      const std::string& matlabUniqueName, const std::string& wrapperName,
      const TypeAttributesTable& typeAttributes,
      std::vector<std::string>& functionNames) const;

private:

  /// Emit C++ code
  std::string wrapper_fragment(FileWriter& wrapperFile,
      const std::string& cppClassName, const std::string& matlabUniqueName,
      int overload, int id, const TypeAttributesTable& typeAttributes,
      const Qualified& instName) const; ///< cpp wrapper
};

} // \namespace wrap

