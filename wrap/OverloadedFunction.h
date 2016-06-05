/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file OverloadedFunction.h
 * @brief Function that can overload its arguments only
 * @author Frank Dellaert
 * @date Nov 13, 2014
 **/

#pragma once

#include "Function.h"
#include "Argument.h"

namespace wrap {

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

  void push_back(const ArgumentList& args) {
    argLists_.push_back(args);
  }

  std::vector<ArgumentList> expandArgumentListsTemplate(
      const TemplateSubstitution& ts) const {
    std::vector<ArgumentList> result;
    for(const ArgumentList& argList: argLists_) {
      ArgumentList instArgList = argList.expandTemplate(ts);
      result.push_back(instArgList);
    }
    return result;
  }

  /// Expand templates, imperative !
  virtual void ExpandTemplate(const TemplateSubstitution& ts) {
    argLists_ = expandArgumentListsTemplate(ts);
  }

  void verifyArguments(const std::vector<std::string>& validArgs,
      const std::string s) const {
    for(const ArgumentList& argList: argLists_) {
      for(Argument arg: argList) {
        std::string fullType = arg.type.qualifiedName("::");
        if (find(validArgs.begin(), validArgs.end(), fullType)
            == validArgs.end())
          throw DependencyMissing(fullType, "checking argument of " + s);
      }
    }
  }

  friend std::ostream& operator<<(std::ostream& os,
      const ArgumentOverloads& overloads) {
    for(const ArgumentList& argList: overloads.argLists_)
      os << argList << std::endl;
    return os;
  }

};

class OverloadedFunction: public Function, public ArgumentOverloads {

public:

  bool addOverload(const std::string& name, const ArgumentList& args,
      boost::optional<const Qualified> instName = boost::none, bool verbose =
          false) {
    bool first = initializeOrCheck(name, instName, verbose);
    ArgumentOverloads::push_back(args);
    return first;
  }

private:

};

// Templated checking functions
// TODO: do this via polymorphism, use transform ?

template<class F>
static std::map<std::string, F> expandMethodTemplate(
    const std::map<std::string, F>& methods, const TemplateSubstitution& ts) {
  std::map<std::string, F> result;
  typedef std::pair<const std::string, F> NamedMethod;
  for(NamedMethod namedMethod: methods) {
    F instMethod = namedMethod.second;
    instMethod.expandTemplate(ts);
    namedMethod.second = instMethod;
    result.insert(namedMethod);
  }
  return result;
}

template<class F>
inline void verifyArguments(const std::vector<std::string>& validArgs,
    const std::map<std::string, F>& vt) {
  typedef typename std::map<std::string, F>::value_type NamedMethod;
  for(const NamedMethod& namedMethod: vt)
    namedMethod.second.verifyArguments(validArgs);
}

} // \namespace wrap

