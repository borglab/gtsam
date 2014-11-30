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

#include "TemplateSubstitution.h"
#include "FileWriter.h"
#include "ReturnValue.h"

namespace wrap {

/// Argument class
struct Argument {
  Qualified type;
  std::string name;
  bool is_const, is_ref, is_ptr;

  Argument() :
      is_const(false), is_ref(false), is_ptr(false) {
  }

  Argument(const Qualified& t, const std::string& n) :
      type(t), name(n), is_const(false), is_ref(false), is_ptr(false) {
  }

  bool operator==(const Argument& other) const {
    return type == other.type && name == other.name
        && is_const == other.is_const && is_ref == other.is_ref
        && is_ptr == other.is_ptr;
  }

  Argument expandTemplate(const TemplateSubstitution& ts) const;

  /// return MATLAB class for use in isa(x,class)
  std::string matlabClass(const std::string& delim = "") const;

  /// Check if will be unwrapped using scalar login in wrap/matlab.h
  bool isScalar() const;

  /// MATLAB code generation, MATLAB to C++
  void matlab_unwrap(FileWriter& file, const std::string& matlabName) const;

  /**
   * emit checking argument to MATLAB proxy
   * @param proxyFile output stream
   */
  void proxy_check(FileWriter& proxyFile, const std::string& s) const;

  friend std::ostream& operator<<(std::ostream& os, const Argument& arg) {
    os << (arg.is_const ? "const " : "") << arg.type << (arg.is_ptr ? "*" : "")
        << (arg.is_ref ? "&" : "");
    return os;
  }

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

  ArgumentList expandTemplate(const TemplateSubstitution& ts) const;

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
   * emit checking arguments to MATLAB proxy
   * @param proxyFile output stream
   */
  void proxy_check(FileWriter& proxyFile) const;

  /// Output stream operator
  friend std::ostream& operator<<(std::ostream& os,
      const ArgumentList& argList) {
    os << "(";
    if (argList.size() > 0)
      os << argList.front();
    if (argList.size() > 1)
      for (size_t i = 1; i < argList.size(); i++)
        os << ", " << argList[i];
    os << ")";
    return os;
  }

};

} // \namespace wrap

