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

using namespace std;

namespace wrap {

	Class TemplateInstantiationTypedef::findAndExpand(const vector<Class>& classes) const {
		// Find matching class
		std::vector<Class>::const_iterator clsIt = classes.end();
		for(std::vector<Class>::const_iterator it = classes.begin(); it != classes.end(); ++it) {
			if(it->name == className && it->namespaces == classNamespaces && it->templateArgs.size() == typeList.size()) {
				clsIt = it;
				break;
			}
		}

		if(clsIt == classes.end())
			throw DependencyMissing(wrap::qualifiedName("::", classNamespaces, className),
			"instantiation into typedef name " + wrap::qualifiedName("::", namespaces, name) +
			".  Ensure that the typedef provides the correct number of template arguments.");

		// Instantiate it
		Class classInst = *clsIt;
		for(size_t i = 0; i < typeList.size(); ++i)
			classInst = classInst.expandTemplate(classInst.templateArgs[i], typeList[i], namespaces, name);

		// Fix class properties
		classInst.name = name;
		classInst.templateArgs.clear();
		classInst.typedefName = clsIt->qualifiedName("::") + "<";
		if(typeList.size() > 0)
			classInst.typedefName += wrap::qualifiedName("::", typeList[0]);
		for(size_t i = 1; i < typeList.size(); ++i)
			classInst.typedefName += (", " + wrap::qualifiedName("::", typeList[i]));
		classInst.typedefName += ">";
		classInst.namespaces = namespaces;

		return classInst;
	}

}