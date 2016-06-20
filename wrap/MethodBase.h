/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MethodBase.h
 * @brief describes and generates code for static methods
 * @author Frank Dellaert
 * @author Alex Cunningham
 * @author Richard Roberts
 **/

#pragma once

#include "FullyOverloadedFunction.h"

namespace wrap {

/// MethodBase class
struct MethodBase: public FullyOverloadedFunction {

  typedef const std::string& Str;

  // emit a list of comments, one for each overload
  void comment_fragment(FileWriter& proxyFile) const {
    SignatureOverloads::comment_fragment(proxyFile, matlabName());
  }

  void verifyArguments(const std::vector<std::string>& validArgs) const {
    SignatureOverloads::verifyArguments(validArgs, name_);
  }

  void verifyReturnTypes(const std::vector<std::string>& validtypes) const {
    SignatureOverloads::verifyReturnTypes(validtypes, name_);
  }

  // MATLAB code generation
  // classPath is class directory, e.g., ../matlab/@Point2
  void proxy_wrapper_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
      Str cppClassName, Str matlabQualName, Str matlabUniqueName,
      Str wrapperName, const TypeAttributesTable& typeAttributes,
      std::vector<std::string>& functionNames) const;

  // emit python wrapper
  void python_wrapper(FileWriter& wrapperFile, Str className) const;

protected:

  virtual void proxy_header(FileWriter& proxyFile) const = 0;

  std::string wrapper_fragment(FileWriter& wrapperFile, Str cppClassName,
      Str matlabUniqueName, int overload, int id,
      const TypeAttributesTable& typeAttributes) const; ///< cpp wrapper

  virtual std::string wrapper_call(FileWriter& wrapperFile, Str cppClassName,
      Str matlabUniqueName, const ArgumentList& args) const = 0;
};

} // \namespace wrap

