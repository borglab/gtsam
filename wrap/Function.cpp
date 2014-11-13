/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Function.ccp
 * @author Frank Dellaert
 * @date Nov 13, 2014
 **/

#include "Function.h"
#include "utilities.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Function::addOverload(bool verbose, const std::string& name,
    const ArgumentList& args, const ReturnValue& retVal,
    const Qualified& instName) {

  // Check if this overload is give to the correct method
  if (name_.empty())
    name_ = name;
  else if (name_ != name)
    throw std::runtime_error(
        "Function::addOverload: tried to add overload with name " + name
            + " instead of expected " + name_);

  // Check if this overload is give to the correct method
  if (templateArgValue_.empty())
    templateArgValue_ = instName;
  else if (templateArgValue_ != instName)
    throw std::runtime_error(
        "Function::addOverload: tried to add overload with template argument "
            + instName.qualifiedName(":") + " instead of expected "
            + templateArgValue_.qualifiedName(":"));

  verbose_ = verbose;
  argLists.push_back(args);
  returnVals.push_back(retVal);
}

/* ************************************************************************* */
vector<ArgumentList> Function::expandArgumentListsTemplate(
    const string& templateArg, const Qualified& qualifiedType,
    const Qualified& expandedClass) const {
  vector<ArgumentList> result;
  BOOST_FOREACH(const ArgumentList& argList, argLists) {
    ArgumentList instArgList = argList.expandTemplate(templateArg,
        qualifiedType, expandedClass);
    result.push_back(instArgList);
  }
  return result;
}

/* ************************************************************************* */
