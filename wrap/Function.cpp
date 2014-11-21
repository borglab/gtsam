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
bool Function::initializeOrCheck(const std::string& name,
    const Qualified& instName, bool verbose) {

  if (name.empty())
    throw std::runtime_error(
        "Function::initializeOrCheck called with empty name");

  // Check if this overload is give to the correct method
  if (name_.empty()) {
    name_ = name;
    templateArgValue_ = instName;
    verbose_ = verbose;
    return true;
  } else {
    if (name_ != name || templateArgValue_ != instName || verbose_ != verbose)
      throw std::runtime_error(
          "Function::initializeOrCheck called with different arguments:  with name "
              + name + " instead of expected " + name_
              + ", or with template argument " + instName.qualifiedName(":")
              + " instead of expected " + templateArgValue_.qualifiedName(":"));

    return false;
  }
}

/* ************************************************************************* */
