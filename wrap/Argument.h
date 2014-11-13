/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Argument.h
 * @brief arguments to constructors and methods
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#pragma once

#include "Qualified.h"
#include "FileWriter.h"
#include "ReturnValue.h"

namespace wrap {

/// Argument class
struct Argument {
  Qualified type;
  bool is_const, is_ref, is_ptr;
  std::string name;

  Argument() :
      is_const(false), is_ref(false), is_ptr(false) {
  }

  Argument expandTemplate(const std::string& templateArg,
      const Qualified& qualifiedType, const Qualified& expandedClass) const;

  /// return MATLAB class for use in isa(x,class)
  std::string matlabClass(const std::string& delim = "") const;

  /// Check if will be unwrapped using scalar login in wrap/matlab.h
  bool isScalar() const;

  /// MATLAB code generation, MATLAB to C++
  void matlab_unwrap(FileWriter& file, const std::string& matlabName) const;
};

/// Argument list is just a container with Arguments
struct ArgumentList: public std::vector<Argument> {

  /// create a comma-separated string listing all argument types (not used)
  std::string types() const;

  /// create a short "signature" string
  std::string signature() const;

  /// create a comma-separated string listing all argument names, used in m-files
  std::string names() const;

  /// Check if all arguments scalar
  bool allScalar() const;

  ArgumentList expandTemplate(const std::string& templateArg,
      const Qualified& qualifiedType, const Qualified& expandedClass) const;

  // MATLAB code generation:

  /**
   * emit code to unwrap arguments
   * @param file output stream
   * @param start initial index for input array, set to 1 for method
   */
  void matlab_unwrap(FileWriter& file, int start = 0) const; // MATLAB to C++

  /**
   * emit MATLAB prototype
   * @param file output stream
   * @param name of method or function
   */
  void emit_prototype(FileWriter& file, const std::string& name) const;

  /**
   * emit emit MATLAB call to proxy
   * @param proxyFile output stream
   * @param returnVal the return value
   * @param wrapperName of method or function
   * @param staticMethod flag to emit "this" in call
   */
  void emit_call(FileWriter& proxyFile, const ReturnValue& returnVal,
      const std::string& wrapperName, int id, bool staticMethod = false) const;

  /**
   * emit conditional MATLAB call to proxy (checking arguments first)
   * @param proxyFile output stream
   * @param returnVal the return value
   * @param wrapperName of method or function
   * @param staticMethod flag to emit "this" in call
   */
  void emit_conditional_call(FileWriter& proxyFile,
      const ReturnValue& returnVal, const std::string& wrapperName, int id,
      bool staticMethod = false) const;
};

template<class T>
inline void verifyArguments(const std::vector<std::string>& validArgs,
    const std::map<std::string, T>& vt) {
  typedef typename std::map<std::string, T>::value_type NamedMethod;
  BOOST_FOREACH(const NamedMethod& namedMethod, vt) {
    const T& t = namedMethod.second;
    BOOST_FOREACH(const ArgumentList& argList, t.argLists) {
      BOOST_FOREACH(Argument arg, argList) {
        std::string fullType = arg.type.qualifiedName("::");
        if (find(validArgs.begin(), validArgs.end(), fullType)
            == validArgs.end())
          throw DependencyMissing(fullType, t.name_);
      }
    }
  }
}

} // \namespace wrap

