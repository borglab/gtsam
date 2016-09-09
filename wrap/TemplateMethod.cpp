/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TemplateMethod.ccp
 * @author Duy-Nguyen Ta
 **/

#include "TemplateMethod.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void TemplateMethod::emit_cython_pxd(FileWriter& file) const {
  for(size_t i = 0; i < nrOverloads(); ++i) {
    file.oss << "\t\t";
    returnVals_[i].emit_cython_pxd(file);
    file.oss << name_ << "[" << argName << "]" << "(";
    argumentList(i).emit_cython_pxd(file);
    file.oss << ")\n";
  }
}

/* ************************************************************************* */
bool TemplateMethod::addOverload(Str name, const ArgumentList& args,
    const ReturnValue& retVal, bool is_const,
    std::string _argName, bool verbose) {
  argName = _argName;
  bool first = MethodBase::addOverload(name, args, retVal, boost::none, verbose);
  if (first)
    is_const_ = is_const;
  else if (is_const && !is_const_)
    throw std::runtime_error(
        "Method::addOverload now designated as const whereas before it was not");
  else if (!is_const && is_const_)
    throw std::runtime_error(
        "Method::addOverload now designated as non-const whereas before it was");
  return first;
}

/* ************************************************************************* */
