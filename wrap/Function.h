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
  std::vector<ArgumentList> argLists;
  std::vector<ReturnValue> returnVals;

  // The first time this function is called, it initializes the class members
  // with those in rhs, but in subsequent calls it adds additional argument
  // lists as function overloads.
  void addOverload(bool verbose, const std::string& name,
      const ArgumentList& args, const ReturnValue& retVal,
      const Qualified& instName = Qualified());

  std::vector<ArgumentList> expandArgumentListsTemplate(
      const std::string& templateArg, const Qualified& qualifiedType,
      const Qualified& expandedClass) const;
};

// Templated checking functions
// TODO: do this via polymorphism ?

template<class FUNCTION>
FUNCTION expandMethodTemplate(const FUNCTION& method,
    const std::string& templateArg, const Qualified& qualifiedType,
    const Qualified& expandedClass) {
  // Create new instance
  FUNCTION instMethod = method;
  // substitute template in arguments
  instMethod.argLists = method.expandArgumentListsTemplate(templateArg,
      qualifiedType, expandedClass);
  // do the same for return types
  instMethod.returnVals = ReturnValue::ExpandTemplate(method.returnVals,
      templateArg, qualifiedType, expandedClass);
  // return new method
  return instMethod;
}

// TODO use transform
template<class FUNCTION>
static std::map<std::string, FUNCTION> expandMethodTemplate(
    const std::map<std::string, FUNCTION>& methods,
    const std::string& templateArg, const Qualified& qualifiedType,
    const Qualified& expandedClass) {
  std::map<std::string, FUNCTION> result;
  typedef std::pair<const std::string, FUNCTION> NamedMethod;
  BOOST_FOREACH(NamedMethod namedMethod, methods) {
    namedMethod.second = expandMethodTemplate(namedMethod.second, templateArg,
        qualifiedType, expandedClass);
    result.insert(namedMethod);
  }
  return result;
}
template<class T>
inline void verifyReturnTypes(const std::vector<std::string>& validtypes,
    const std::map<std::string, T>& vt) {
  typedef typename std::map<std::string, T>::value_type NamedMethod;
  BOOST_FOREACH(const NamedMethod& namedMethod, vt) {
    const T& t = namedMethod.second;
    BOOST_FOREACH(const ReturnValue& retval, t.returnVals) {
      retval.type1.verify(validtypes, t.name_);
      if (retval.isPair)
        retval.type2.verify(validtypes, t.name_);
    }
  }
}

} // \namespace wrap

