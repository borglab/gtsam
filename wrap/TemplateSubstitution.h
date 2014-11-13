/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TemplateSubstitution.h
 * @brief Auxiliary class for template sunstitutions
 * @author Frank Dellaert
 * @date Nov 13, 2014
 **/

#pragma once

#include "Qualified.h"
#include <string>

namespace wrap {

/**
 * e.g. TemplateSubstitution("T", gtsam::Point2, gtsam::PriorFactorPoint2)
 */
struct TemplateSubstitution {
  TemplateSubstitution(const std::string& a, const Qualified& t,
      const Qualified& e) :
      templateArg(a), qualifiedType(t), expandedClass(e) {
  }
  std::string templateArg;
  Qualified qualifiedType, expandedClass;
};

} // \namespace wrap

