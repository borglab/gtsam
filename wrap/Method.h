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

#include "StaticMethod.h"

namespace wrap {

/// Method class
class Method: public StaticMethod {

  bool is_const_;

public:

  typedef const std::string& Str;

  bool addOverload(Str name, const ArgumentList& args,
      const ReturnValue& retVal, bool is_const, const Qualified& instName =
          Qualified(), bool verbose = false);

  virtual bool isStatic() const {
    return false;
  }

  virtual bool isConst() const {
    return is_const_;
  }

  friend std::ostream& operator<<(std::ostream& os, const Method& m) {
    for (size_t i = 0; i < m.nrOverloads(); i++)
      os << m.returnVals_[i] << " " << m.name_ << m.argLists_[i];
    return os;
  }

private:

  // Emit method header
  void proxy_header(FileWriter& proxyFile) const;

  virtual std::string wrapper_call(FileWriter& wrapperFile, Str cppClassName,
      Str matlabUniqueName, const ArgumentList& args,
      const ReturnValue& returnVal, const TypeAttributesTable& typeAttributes,
      const Qualified& instName) const;
};

} // \namespace wrap

