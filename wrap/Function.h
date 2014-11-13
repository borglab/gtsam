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
#include "ReturnValue.h"
#include "TypeAttributesTable.h"

#include <string>
#include <list>

namespace wrap {

/// Function class
struct Function {

  /// Constructor creates empty object
  Function(bool verbose = true) :
      verbose_(verbose) {
  }

  Function(const std::string& name, bool verbose = true) :
      verbose_(verbose), name_(name) {
  }

  bool verbose_;
  std::string name_; ///< name of method
  Qualified templateArgValue_; ///< value of template argument if applicable

  // The first time this function is called, it initializes the class members
  // with those in rhs, but in subsequent calls it adds additional argument
  // lists as function overloads.
  void addOverload(bool verbose, const std::string& name,
      const Qualified& instName = Qualified());
};

/**
 * ArgumentList Overloads
 */
class ArgumentOverloads {

protected:

  std::vector<ArgumentList> argLists_;

public:

  size_t nrOverloads() const {
    return argLists_.size();
  }

  const ArgumentList& argumentList(size_t i) const {
    return argLists_.at(i);
  }

  void addOverload(const ArgumentList& args) {
    argLists_.push_back(args);
  }

  std::vector<ArgumentList> expandArgumentListsTemplate(
      const TemplateSubstitution& ts) const;

  /// Expand templates, imperative !
  virtual void ExpandTemplate(const TemplateSubstitution& ts) {
    argLists_ = expandArgumentListsTemplate(ts);
  }

  void verifyArguments(const std::vector<std::string>& validArgs,
      const std::string s) const {
    BOOST_FOREACH(const ArgumentList& argList, argLists_) {
      BOOST_FOREACH(Argument arg, argList) {
        std::string fullType = arg.type.qualifiedName("::");
        if (find(validArgs.begin(), validArgs.end(), fullType)
            == validArgs.end())
          throw DependencyMissing(fullType, s);
      }
    }
  }

};

/**
 * Signature Overload (including return value)
 */
class SignatureOverloads: public ArgumentOverloads {

protected:

  std::vector<ReturnValue> returnVals_;

public:

  const ReturnValue& returnValue(size_t i) const {
    return returnVals_.at(i);
  }

  void addOverload(const ArgumentList& args, const ReturnValue& retVal) {
    argLists_.push_back(args);
    returnVals_.push_back(retVal);
  }

  void verifyReturnTypes(const std::vector<std::string>& validtypes,
      const std::string& s) const {
    BOOST_FOREACH(const ReturnValue& retval, returnVals_) {
      retval.type1.verify(validtypes, s);
      if (retval.isPair)
        retval.type2.verify(validtypes, s);
    }
  }

  // TODO use transform ?
  std::vector<ReturnValue> ExpandReturnValuesTemplate(
      const TemplateSubstitution& ts) const {
    std::vector<ReturnValue> result;
    BOOST_FOREACH(const ReturnValue& retVal, returnVals_) {
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
    returnVals_ = ExpandReturnValuesTemplate(ts);
  }

  // emit a list of comments, one for each overload
  void usage_fragment(FileWriter& proxyFile, const std::string& name) const {
    unsigned int argLCount = 0;
    BOOST_FOREACH(ArgumentList argList, argLists_) {
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
    BOOST_FOREACH(ArgumentList argList, argLists_) {
      proxyFile.oss << "%";
      argList.emit_prototype(proxyFile, name);
      proxyFile.oss << " : returns " << returnVals_[i++].return_type(false)
          << std::endl;
    }
  }

};

// Templated checking functions
// TODO: do this via polymorphism ?

template<class F>
F expandMethodTemplate(F& method, const TemplateSubstitution& ts) {
  F instMethod = method;
  method.expandTemplate(ts);
  return instMethod;
}

// TODO use transform
template<class F>
static std::map<std::string, F> expandMethodTemplate(
    const std::map<std::string, F>& methods, const TemplateSubstitution& ts) {
  std::map<std::string, F> result;
  typedef std::pair<const std::string, F> NamedMethod;
  BOOST_FOREACH(NamedMethod namedMethod, methods) {
    namedMethod.second = expandMethodTemplate(namedMethod.second, ts);
    result.insert(namedMethod);
  }
  return result;
}
template<class F>
inline void verifyReturnTypes(const std::vector<std::string>& validTypes,
    const std::map<std::string, F>& vt) {
  typedef typename std::map<std::string, F>::value_type NamedMethod;
  BOOST_FOREACH(const NamedMethod& namedMethod, vt) {
    const F& t = namedMethod.second;
    t.verifyReturnTypes(validTypes, t.name_);
  }
}

} // \namespace wrap

