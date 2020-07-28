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

#include "MethodBase.h"

namespace wrap {

/// Method class
class Method: public MethodBase {

protected:
  bool is_const_;

public:

  typedef const std::string& Str;

  bool addOverload(Str name, const ArgumentList& args,
      const ReturnValue& retVal, bool is_const,
      boost::optional<const Qualified> instName = boost::none, bool verbose =
          false);

  bool isStatic() const override {
    return false;
  }

  virtual bool isConst() const {
    return is_const_;
  }

  bool isSameModifiers(const Method& other) const {
      return is_const_ == other.is_const_ &&
             ((templateArgValue_ && other.templateArgValue_) ||
              (!templateArgValue_ && !other.templateArgValue_));
  }

  friend std::ostream& operator<<(std::ostream& os, const Method& m) {
    for (size_t i = 0; i < m.nrOverloads(); i++)
      os << m.returnVals_[i] << " " << m.name_ << m.argLists_[i];
    return os;
  }

  void emit_cython_pxd(FileWriter& file, const Class& cls) const;
  void emit_cython_pyx(FileWriter& file, const Class& cls) const;
  void emit_cython_pyx_no_overload(FileWriter& file, const Class& cls) const;

private:

  // Emit method header
  void proxy_header(FileWriter& proxyFile) const override;

  std::string wrapper_call(FileWriter& wrapperFile, Str cppClassName,
      Str matlabUniqueName, const ArgumentList& args) const override;
};

} // \namespace wrap

