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

#include "MethodBase.h"

namespace wrap {

/// StaticMethod class
struct StaticMethod: public MethodBase {

  typedef const std::string& Str;

  friend std::ostream& operator<<(std::ostream& os, const StaticMethod& m) {
    for (size_t i = 0; i < m.nrOverloads(); i++)
      os << "static " << m.returnVals_[i] << " " << m.name_ << m.argLists_[i];
    return os;
  }

  void emit_cython_pxd(FileWriter& file, const Class& cls) const;
  void emit_cython_wrapper_pxd(FileWriter& file, const Class& cls) const;
  void emit_cython_pyx(FileWriter& file, const Class& cls) const;
  void emit_cython_pyx_no_overload(FileWriter& file, const Class& cls) const;

protected:

  void proxy_header(FileWriter& proxyFile) const override;

  std::string wrapper_call(FileWriter& wrapperFile, Str cppClassName,
      Str matlabUniqueName, const ArgumentList& args) const override;
};

} // \namespace wrap

