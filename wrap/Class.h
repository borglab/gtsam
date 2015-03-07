/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Class.h
 * @brief describe the C++ class that is being wrapped
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#pragma once

#include <string>
#include <map>

#include "Constructor.h"
#include "Deconstructor.h"
#include "Method.h"
#include "StaticMethod.h"
#include "TypeAttributesTable.h"
#include <boost/algorithm/string.hpp>

namespace wrap {

/// Class has name, constructors, methods
struct Class {
	typedef std::map<std::string, Method> Methods;
	typedef std::map<std::string, StaticMethod> StaticMethods;

  /// Constructor creates an empty class
  Class(bool verbose=true) : isVirtual(false), verbose_(verbose) {}

	// Then the instance variables are set directly by the Module constructor
  std::string name;                         ///< Class name
	std::vector<std::string> templateArgs;    ///< Template arguments
	std::string typedefName;                  ///< The name to typedef *from*, if this class is actually a typedef, i.e. typedef [typedefName] [name]
	bool isVirtual;                           ///< Whether the class is part of a virtual inheritance chain
	std::vector<std::string> qualifiedParent; ///< The *single* parent - the last string is the parent class name, preceededing elements are a namespace stack
  Methods methods;                          ///< Class methods
  StaticMethods static_methods;             ///< Static methods
  std::vector<std::string> namespaces;      ///< Stack of namespaces
  Constructor constructor;                  ///< Class constructors
	Deconstructor deconstructor;              ///< Deconstructor to deallocate C++ object
  bool verbose_;                            ///< verbose flag

  // And finally MATLAB code is emitted, methods below called by Module::matlab_code
  void matlab_proxy(const std::string& toolboxPath, const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
		FileWriter& wrapperFile, std::vector<std::string>& functionNames) const;          ///< emit proxy class
  std::string qualifiedName(const std::string& delim = "") const; ///< creates a namespace-qualified name, optional delimiter

	std::vector<Class> expandTemplate(const std::string& templateArg, const std::vector<std::vector<std::string> >& instantiations) const;
	Class expandTemplate(const std::string& templateArg, const std::vector<std::string>& instantiation, const std::vector<std::string>& expandedClassNamespace, const std::string& expandedClassName) const;

	// The typedef line for this class, if this class is a typedef, otherwise returns an empty string.
	std::string getTypedef() const;


private:
	void pointer_constructor_fragments(FileWriter& proxyFile, FileWriter& wrapperFile, const std::string& wrapperName, std::vector<std::string>& functionNames) const;
    void comment_fragment(FileWriter& proxyFile) const;
};

} // \namespace wrap

