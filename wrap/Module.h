/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Module.h
 * @brief describes module to be wrapped
 * @author Frank Dellaert
 * @author Richard Roberts
 **/

#pragma once

#include <string>
#include <vector>
#include <map>

#include "Class.h"
#include "GlobalFunction.h"
#include "TemplateInstantiationTypedef.h"
#include "ForwardDeclaration.h"

namespace wrap {

/**
 * A module just has a name and a list of classes
 */
struct Module {

  typedef std::map<std::string, GlobalFunction> GlobalFunctions;
  typedef std::map<std::string, Method> Methods;

  std::string name;         ///< module name
  bool verbose;            ///< verbose flag
  std::vector<Class> classes; ///< list of classes
  std::vector<TemplateInstantiationTypedef> templateInstantiationTypedefs; ///< list of template instantiations
  std::vector<ForwardDeclaration> forward_declarations;
  std::vector<std::string> includes;        ///< Include statements
  GlobalFunctions global_functions;

  /// constructor that parses interface file
  Module(const std::string& interfacePath,
   const std::string& moduleName,
   bool enable_verbose=true);

  /// Dummy constructor that does no parsing - use only for testing
  Module(const std::string& moduleName, bool enable_verbose=true);

  //Recursive method to append all methods inhereted from parent classes
  std::map<std::string, Method> appendInheretedMethods(const Class& cls, const std::vector<Class>& classes);

  /// MATLAB code generation:
  void matlab_code(
       const std::string& path,
       const std::string& headerPath) const; // FIXME: headerPath not actually used?

  void finish_wrapper(FileWriter& file, const std::vector<std::string>& functionNames) const;

  void generateIncludes(FileWriter& file) const;

  /// non-const function that performs parsing - typically called by constructor
  /// Throws exception on failure
  void parseMarkup(const std::string& data);

private:
  static std::vector<Class> ExpandTypedefInstantiations(const std::vector<Class>& classes, const std::vector<TemplateInstantiationTypedef> instantiations);
  static std::vector<std::string> GenerateValidTypes(const std::vector<Class>& classes, const std::vector<ForwardDeclaration> forwardDeclarations);
  static void WriteCollectorsAndCleanupFcn(FileWriter& wrapperFile, const std::string& moduleName, const std::vector<Class>& classes);
  static void WriteRTTIRegistry(FileWriter& wrapperFile, const std::string& moduleName, const std::vector<Class>& classes);
};

} // \namespace wrap
