/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file StaticMethod.h
 * @brief describes and generates code for static methods
 * @author Frank Dellaert
 * @author Alex Cunningham
 * @author Richard Roberts
 **/

#pragma once

#include "Function.h"

namespace wrap {

/// StaticMethod class
struct StaticMethod: public Function, public SignatureOverloads {

  typedef const std::string& Str;

  /// Constructor creates empty object
  StaticMethod(bool verbosity = true) :
      Function(verbosity) {
  }

  virtual bool isStatic() const {
    return true;
  }

  void addOverload(bool verbose, Str name, const ArgumentList& args,
      const ReturnValue& retVal, const Qualified& instName);

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

  friend std::ostream& operator<<(std::ostream& os, const StaticMethod& m) {
    for (size_t i = 0; i < m.nrOverloads(); i++)
      os << "static " << m.returnVals_[i] << " " << m.name_ << m.argLists_[i];
    return os;
  }

protected:

  virtual void proxy_header(FileWriter& proxyFile) const;

  std::string wrapper_fragment(FileWriter& wrapperFile, Str cppClassName,
      Str matlabUniqueName, int overload, int id,
      const TypeAttributesTable& typeAttributes, const Qualified& instName =
          Qualified()) const; ///< cpp wrapper

  virtual std::string wrapper_call(FileWriter& wrapperFile, Str cppClassName,
      Str matlabUniqueName, const ArgumentList& args,
      const ReturnValue& returnVal, const TypeAttributesTable& typeAttributes,
      const Qualified& instName) const;
};

} // \namespace wrap

