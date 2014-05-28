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

#include "FileWriter.h"
#include "ReturnValue.h"

#include <string>
#include <vector>

namespace wrap {

/// Argument class
struct Argument {
  bool is_const, is_ref, is_ptr;
  std::string type;
  std::string name;
  std::vector<std::string> namespaces;

  Argument() :
      is_const(false), is_ref(false), is_ptr(false) {
  }

  /// return MATLAB class for use in isa(x,class)
  std::string matlabClass(const std::string& delim = "") const;

  /// Check if will be unwrapped using scalar login in wrap/matlab.h
  bool isScalar() const;

  /// adds namespaces to type
  std::string qualifiedType(const std::string& delim = "") const;

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
   * emit emit MATLAB call to wrapper
   * @param file output stream
   * @param returnVal the return value
   * @param wrapperName of method or function
   * @param staticMethod flag to emit "this" in call
   */
  void emit_call(FileWriter& file, const ReturnValue& returnVal,
      const std::string& wrapperName, int id, bool staticMethod = false) const;

  /**
   * emit conditional MATLAB call to wrapper (checking arguments first)
   * @param file output stream
   * @param returnVal the return value
   * @param wrapperName of method or function
   * @param staticMethod flag to emit "this" in call
   */
  void emit_conditional_call(FileWriter& file, const ReturnValue& returnVal,
      const std::string& wrapperName, int id, bool staticMethod = false) const;
};

} // \namespace wrap

