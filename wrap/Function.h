/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Function.h
 * @brief Base class for global functions and methods
 * @author Frank Dellaert
 * @date Nov 13, 2014
 **/

#pragma once

#include "Argument.h"

namespace wrap {

/// Function class
class Function {

protected:

  std::string name_; ///< name of method
  Qualified templateArgValue_; ///< value of template argument if applicable
  bool verbose_;

public:

  /**
   * @brief first time, fill in instance variables, otherwise check if same
   * @return true if first time, false thereafter
   */
  bool initializeOrCheck(const std::string& name, const Qualified& instName =
      Qualified(), bool verbose = false);

  std::string name() const {
    return name_;
  }

  /// Only Methods are non-static
  virtual bool isStatic() const {
    return true;
  }

  std::string matlabName() const {
    if (templateArgValue_.empty())
      return name_;
    else
      return name_ + templateArgValue_.name();
  }

  /// Emit function call to MATLAB (no argument checking)
  void emit_call(FileWriter& proxyFile, const ReturnValue& returnVal,
      const std::string& wrapperName, int id) const;

  /// Emit checking arguments and function call to MATLAB
  void emit_conditional_call(FileWriter& proxyFile,
      const ReturnValue& returnVal, const ArgumentList& args,
      const std::string& wrapperName, int id) const;

};

} // \namespace wrap

