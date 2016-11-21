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

  std::string pyx_resolveOverloadParams(const ArgumentList& args, bool isVoid) const {
    std::string s;
    s += "\t\tif len(args)+len(kwargs) !=" + std::to_string(args.size()) + ":\n";
    s += "\t\t\treturn False";
    s += (!isVoid) ? ", None\n" : "\n";
    if (args.size() > 0) {
      s += "\t\t__params = kwargs.copy()\n";
      s += "\t\t__names = [" + args.pyx_paramsList() + "]\n";
      s += "\t\tfor i in range(len(args)):\n"; 
      s += "\t\t\t__params[__names[i]] = args[i]\n";
      s += "\t\ttry:\n";
      s += args.pyx_castParamsToPythonType();
      s += "\t\texcept:\n";
      s += "\t\t\treturn False";
      s += (!isVoid) ? ", None\n" : "\n";
    }
    return s;
  }

  /// if two overloading methods have the same number of arguments, they have
  /// to be resolved via keyword args
  std::string pyx_checkDuplicateNargsKwArgs() const {
    std::unordered_set<size_t> nargsSet;
    std::vector<size_t> nargsDuplicates;
    for (size_t i = 0; i < nrOverloads(); ++i) {
      size_t nargs = argumentList(i).size();
      if (nargsSet.find(nargs) != nargsSet.end())
        nargsDuplicates.push_back(nargs);
      else
        nargsSet.insert(nargs);
    }

    std::string s;
    if (nargsDuplicates.size() > 0) {
      s += "\t\tif len(kwargs)==0 and len(args)+len(kwargs) in [";
      for (size_t i = 0; i < nargsDuplicates.size(); ++i) {
        s += std::to_string(nargsDuplicates[i]);
        if (i < nargsDuplicates.size() - 1) s += ",";
      }
      s += "]:\n";
      s += "\t\t\traise TypeError('Overloads with the same number of " 
           "arguments exist. Please use keyword arguments to " 
           "differentiate them!')\n";
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
