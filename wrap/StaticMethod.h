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
struct StaticMethod: public Function {

  /// Constructor creates empty object
  StaticMethod(bool verbosity = true) :
      Function(verbosity) {
  }

  // MATLAB code generation
  // classPath is class directory, e.g., ../matlab/@Point2
  void proxy_wrapper_fragments(FileWriter& proxyFile, FileWriter& wrapperFile,
      const std::string& cppClassName, const std::string& matlabQualName,
      const std::string& matlabUniqueName, const std::string& wrapperName,
      const TypeAttributesTable& typeAttributes,
      std::vector<std::string>& functionNames) const;

private:
  std::string wrapper_fragment(FileWriter& file,
      const std::string& cppClassName, const std::string& matlabUniqueName,
      int overload, int id, const TypeAttributesTable& typeAttributes) const; ///< cpp wrapper
};

} // \namespace wrap

