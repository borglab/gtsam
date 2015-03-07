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

#include "TypeAttributesTable.h"
#include "Class.h"
#include "utilities.h"

#include <boost/foreach.hpp>

using namespace std;

namespace wrap {

	/* ************************************************************************* */
	void TypeAttributesTable::addClasses(const vector<Class>& classes) {
		BOOST_FOREACH(const Class& cls, classes) {
			if(!insert(make_pair(cls.qualifiedName("::"), TypeAttributes(cls.isVirtual))).second)
				throw DuplicateDefinition("class " + cls.qualifiedName("::"));
		}
	}

	/* ************************************************************************* */
	void TypeAttributesTable::addForwardDeclarations(const vector<ForwardDeclaration>& forwardDecls) {
		BOOST_FOREACH(const ForwardDeclaration& fwDec, forwardDecls) {
			if(!insert(make_pair(fwDec.name, TypeAttributes(fwDec.isVirtual))).second)
				throw DuplicateDefinition("class " + fwDec.name);
		}
	}

	/* ************************************************************************* */
	void TypeAttributesTable::checkValidity(const vector<Class>& classes) const {
		BOOST_FOREACH(const Class& cls, classes) {
			// Check that class is virtual if it has a parent
			if(!cls.qualifiedParent.empty() && !cls.isVirtual)
				throw AttributeError(cls.qualifiedName("::"), "Has a base class so needs to be declared virtual, change to 'virtual class "+cls.name+" ...'");
			// Check that parent is virtual as well
			if(!cls.qualifiedParent.empty() && !at(wrap::qualifiedName("::", cls.qualifiedParent)).isVirtual)
				throw AttributeError(wrap::qualifiedName("::", cls.qualifiedParent),
				"Is the base class of " + cls.qualifiedName("::") + ", so needs to be declared virtual");
		}
	}

}