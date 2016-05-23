/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Class.cpp
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#include "TemplateInstantiationTypedef.h"

#include "utilities.h"
#include <iostream>
#include <boost/optional.hpp>

using namespace std;

namespace wrap {

Class TemplateInstantiationTypedef::findAndExpand(
    const vector<Class>& classes) const {
  // Find matching class
  boost::optional<Class const &> matchedClass;
  for(const Class& cls: classes) {
    if (cls.name() == class_.name() && cls.namespaces() == class_.namespaces()
        && cls.templateArgs.size() == typeList.size()) {
      matchedClass.reset(cls);
      break;
    }
  }

  if (!matchedClass)
    throw DependencyMissing(class_.qualifiedName("::"),
        "instantiation into typedef name " + qualifiedName("::")
            + ".  Ensure that the typedef provides the correct number of template arguments.");

  // Instantiate it
  Class classInst = *matchedClass;
  for (size_t i = 0; i < typeList.size(); ++i) {
    TemplateSubstitution ts(classInst.templateArgs[i], typeList[i], *this);
    classInst = classInst.expandTemplate(ts);
  }

  // Fix class properties
  classInst.name_ = name();
  classInst.namespaces_ = namespaces();
  classInst.templateArgs.clear();
  classInst.typedefName = matchedClass->qualifiedName("::") + "<";
  if (typeList.size() > 0)
    classInst.typedefName += typeList[0].qualifiedName("::");
  for (size_t i = 1; i < typeList.size(); ++i)
    classInst.typedefName += (", " + typeList[i].qualifiedName("::"));
  classInst.typedefName += ">";

  return classInst;
}

}
