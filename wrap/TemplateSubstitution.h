/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TemplateSubstitution.h
 * @brief Auxiliary class for template substitutions
 * @author Frank Dellaert
 * @date Nov 13, 2014
 **/

#pragma once

#include "ReturnType.h"
#include <string>
#include <iostream>

namespace wrap {

/**
 * e.g. TemplateSubstitution("T", gtsam::Point2, gtsam::PriorFactorPoint2)
 */
class TemplateSubstitution {

  std::string templateArg_;
  Qualified qualifiedType_, expandedClass_;

public:

  TemplateSubstitution(const std::string& a, const Qualified& t,
      const Qualified& e) :
      templateArg_(a), qualifiedType_(t), expandedClass_(e) {
  }

  std::string expandedClassName() const {
    return expandedClass_.name();
  }

  // Substitute if needed
  Qualified tryToSubstitite(const Qualified& type) const {
    if (type.match(templateArg_))
      return qualifiedType_;
    else if (type.match("This"))
      return expandedClass_;
    else
      return type;
  }

  // Substitute if needed
  ReturnType tryToSubstitite(const ReturnType& type) const {
    ReturnType instType = type;
    if (type.match(templateArg_))
      instType.rename(qualifiedType_);
    else if (type.match("This"))
      instType.rename(expandedClass_);
    return instType;
  }

  friend std::ostream& operator<<(std::ostream& os,
      const TemplateSubstitution& ts) {
    os << ts.templateArg_ << '/' << ts.qualifiedType_.qualifiedName("::")
        << " (" << ts.expandedClass_.qualifiedName("::") << ")";
    return os;
  }

};

} // \namespace wrap

