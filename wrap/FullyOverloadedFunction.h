/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FullyOverloadedFunction.h
 * @brief Function that can be fully overloaded: arguments and return values
 * @author Frank Dellaert
 * @date Nov 13, 2014
 **/

#pragma once

#include "OverloadedFunction.h"
#include <array>

namespace wrap {

/**
 * Signature Overload (including return value)
 */
class SignatureOverloads: public ArgumentOverloads {

public:

  std::vector<ReturnValue> returnVals_;

public:

  const ReturnValue& returnValue(size_t i) const {
    return returnVals_.at(i);
  }

  void push_back(const ArgumentList& args, const ReturnValue& retVal) {
    argLists_.push_back(args);
    returnVals_.push_back(retVal);
  }

  void verifyReturnTypes(const std::vector<std::string>& validtypes,
      const std::string& s) const {
    for(const ReturnValue& retval: returnVals_) {
      retval.type1.verify(validtypes, s);
      if (retval.isPair)
        retval.type2.verify(validtypes, s);
    }
  }

  // TODO use transform ?
  std::vector<ReturnValue> expandReturnValuesTemplate(
      const TemplateSubstitution& ts) const {
    std::vector<ReturnValue> result;
    for(const ReturnValue& retVal: returnVals_) {
      ReturnValue instRetVal = retVal.expandTemplate(ts);
      result.push_back(instRetVal);
    }
    return result;
  }

  /// Expand templates, imperative !
  void expandTemplate(const TemplateSubstitution& ts) {
    // substitute template in arguments
    argLists_ = expandArgumentListsTemplate(ts);
    // do the same for return types
    returnVals_ = expandReturnValuesTemplate(ts);
  }

  // emit a list of comments, one for each overload
  void usage_fragment(FileWriter& proxyFile, const std::string& name) const {
    unsigned int argLCount = 0;
    for(ArgumentList argList: argLists_) {
      argList.emit_prototype(proxyFile, name);
      if (argLCount != nrOverloads() - 1)
        proxyFile.oss << ", ";
      else
        proxyFile.oss << " : returns " << returnValue(0).return_type(false)
            << std::endl;
      argLCount++;
    }
  }

  // emit a list of comments, one for each overload
  void comment_fragment(FileWriter& proxyFile, const std::string& name) const {
    size_t i = 0;
    for(ArgumentList argList: argLists_) {
      proxyFile.oss << "%";
      argList.emit_prototype(proxyFile, name);
      proxyFile.oss << " : returns " << returnVals_[i++].return_type(false)
          << std::endl;
    }
  }

  friend std::ostream& operator<<(std::ostream& os,
      const SignatureOverloads& overloads) {
    for (size_t i = 0; i < overloads.nrOverloads(); i++)
      os << overloads.returnVals_[i] << overloads.argLists_[i] << std::endl;
    return os;
  }

};

class FullyOverloadedFunction: public Function, public SignatureOverloads {

public:

  bool addOverload(const std::string& name, const ArgumentList& args,
      const ReturnValue& retVal, boost::optional<const Qualified> instName =
          boost::none, bool verbose = false) {
    bool first = initializeOrCheck(name, instName, verbose);
    SignatureOverloads::push_back(args, retVal);
    return first;
  }

  // emit cython pyx function call
  std::string pyx_functionCall(const std::string& caller, const std::string& funcName,
                        size_t iOverload) const;

  /// Cython: Rename functions which names are python keywords
  static const std::array<std::string, 2> pythonKeywords;
  static std::string pyRename(const std::string& name) {
    if (std::find(pythonKeywords.begin(), pythonKeywords.end(), name) ==
        pythonKeywords.end())
      return name;
    else
      return name + "_";
  }
};

// Templated checking functions
// TODO: do this via polymorphism, use transform ?

template<class F>
inline void verifyReturnTypes(const std::vector<std::string>& validTypes,
    const std::map<std::string, F>& vt) {
  typedef typename std::map<std::string, F>::value_type NamedMethod;
  for(const NamedMethod& namedMethod: vt)
    namedMethod.second.verifyReturnTypes(validTypes);
}

} // \namespace wrap

