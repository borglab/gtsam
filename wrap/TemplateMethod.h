/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TemplateMethod.h
 * @brief describes and generates code for template methods
 * @author Duy-Nguyen Ta
 **/

#pragma once

#include "Method.h"

namespace wrap {

/// StaticMethod class
struct TemplateMethod: public Method {
  std::string argName; // name of template argument

  void emit_cython_pxd(FileWriter& file, const Class& cls) const;
  bool addOverload(Str name, const ArgumentList& args,
                   const ReturnValue& retVal, bool is_const,
                   std::string argName, bool verbose = false);

  friend std::ostream& operator<<(std::ostream& os, const TemplateMethod& m) {
    for (size_t i = 0; i < m.nrOverloads(); i++)
      os << "template <" << m.argName << "> " << m.returnVals_[i] << " " << m.name_ << m.argLists_[i];
    return os;
  }

};

} // \namespace wrap

