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

#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

#include <iterator>     // std::ostream_iterator
using namespace std;

namespace wrap {

/* ************************************************************************* */
const TypeAttributes& TypeAttributesTable::attributes(const string& key) const {
  try {
    return table_.at(key);
  } catch (const out_of_range& oor) {
    cerr << "TypeAttributesTable::attributes: key " << key
        << " not found. Valid keys are:\n";
    using boost::adaptors::map_keys;
    ostream_iterator<string> out_it(cerr, "\n");
    boost::copy(table_ | map_keys, out_it);
    throw runtime_error("Internal error in wrap");
  }
}

/* ************************************************************************* */
void TypeAttributesTable::addClasses(const vector<Class>& classes) {
  for(const Class& cls: classes) {
    if (!table_.insert(
        make_pair(cls.qualifiedName("::"), TypeAttributes(cls.isVirtual))).second)
      throw DuplicateDefinition("class " + cls.qualifiedName("::"));
  }
}

/* ************************************************************************* */
void TypeAttributesTable::addForwardDeclarations(
    const vector<ForwardDeclaration>& forwardDecls) {
  for(const ForwardDeclaration& fwDec: forwardDecls) {
    if (!table_.insert(make_pair(fwDec.name, TypeAttributes(fwDec.isVirtual))).second)
      throw DuplicateDefinition("class " + fwDec.name);
  }
}

/* ************************************************************************* */
void TypeAttributesTable::checkValidity(const vector<Class>& classes) const {
  for(const Class& cls: classes) {

    boost::optional<string> parent = cls.qualifiedParent();
    if (parent) {

      // Check that class is virtual if it has a parent
      if (!cls.isVirtual)
        throw AttributeError(cls.qualifiedName("::"),
            "Has a base class so needs to be declared virtual, change to 'virtual class "
                + cls.name() + " ...'");

      // Check that parent is virtual as well
      if (!table_.at(*parent).isVirtual)
        throw AttributeError(*parent,
            "Is the base class of " + cls.qualifiedName("::")
                + ", so needs to be declared virtual");
    }
  }
}

}
