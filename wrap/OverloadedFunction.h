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
#include <unordered_set>
namespace wrap {

/**
 * ArgumentList Overloads
 */
class ArgumentOverloads {
public:
  std::vector<ArgumentList> argLists_;

public:
  size_t nrOverloads() const { return argLists_.size(); }

  const ArgumentList& argumentList(size_t i) const { return argLists_.at(i); }

  void push_back(const ArgumentList& args) { argLists_.push_back(args); }

  std::vector<ArgumentList> expandArgumentListsTemplate(
      const TemplateSubstitution& ts) const {
    std::vector<ArgumentList> result;
    for (const ArgumentList& argList : argLists_) {
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
    for (const ArgumentList& argList : argLists_) {
      for (Argument arg : argList) {
        std::string fullType = arg.type.qualifiedName("::");
        if (find(validArgs.begin(), validArgs.end(), fullType) ==
            validArgs.end())
          throw DependencyMissing(fullType, "checking argument of " + s);
      }
    }
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const ArgumentOverloads& overloads) {
    for (const ArgumentList& argList : overloads.argLists_)
      os << argList << std::endl;
    return os;
  }

  std::string pyx_resolveOverloadParams(const ArgumentList& args, bool isVoid,
      size_t indentLevel = 2) const {
    std::string indent;
    for (size_t i = 0; i < indentLevel; ++i)
      indent += "    ";
    std::string s;
    s += indent + "__params = process_args([" + args.pyx_paramsList()
        + "], args, kwargs)\n";
    s += args.pyx_castParamsToPythonType(indent);
    if (args.size() > 0) {
      for (size_t i = 0; i < args.size(); ++i) {
        // For python types we can do the assert after the assignment and save list accesses
        if (args[i].type.isNonBasicType() || args[i].type.isEigen()) {
          std::string param = args[i].name;
          s += indent + "assert isinstance(" + param + ", "
              + args[i].type.pyxArgumentType() + ")";
          if (args[i].type.isEigen()) {
            s += " and " + param + ".ndim == "
                + ((args[i].type.pyxClassName() == "Vector") ? "1" : "2");
          }
          s += "\n";
        }
      }
    }
    return s;
  }
};

class OverloadedFunction : public Function, public ArgumentOverloads {
public:
  bool addOverload(const std::string& name, const ArgumentList& args,
                   boost::optional<const Qualified> instName = boost::none,
                   bool verbose = false) {
    bool first = initializeOrCheck(name, instName, verbose);
    ArgumentOverloads::push_back(args);
    return first;
  }

private:
};

// Templated checking functions
// TODO: do this via polymorphism, use transform ?

template <class F>
static std::map<std::string, F> expandMethodTemplate(
    const std::map<std::string, F>& methods, const TemplateSubstitution& ts) {
  std::map<std::string, F> result;
  typedef std::pair<const std::string, F> NamedMethod;
  for (NamedMethod namedMethod : methods) {
    F instMethod = namedMethod.second;
    instMethod.expandTemplate(ts);
    namedMethod.second = instMethod;
    result.insert(namedMethod);
  }
  return result;
}

template <class F>
inline void verifyArguments(const std::vector<std::string>& validArgs,
                            const std::map<std::string, F>& vt) {
  typedef typename std::map<std::string, F>::value_type NamedMethod;
  for (const NamedMethod& namedMethod : vt)
    namedMethod.second.verifyArguments(validArgs);
}

}  // \namespace wrap
