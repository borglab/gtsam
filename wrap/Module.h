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

#include "Class.h"
#include "GlobalFunction.h"
#include "TemplateInstantiationTypedef.h"
#include "ForwardDeclaration.h"

#include <string>
#include <vector>
#include <map>

namespace wrap {

/**
 * A module just has a name and a list of classes
 */
struct Module {

  // Filled during parsing:
  std::string name; ///< module name
  bool verbose; ///< verbose flag
  std::vector<Class> classes; ///< list of classes
  std::vector<TemplateInstantiationTypedef> templateInstantiationTypedefs; ///< list of template instantiations
  std::vector<ForwardDeclaration> forward_declarations;
  std::vector<std::string> includes; ///< Include statements
  GlobalFunctions global_functions;

  // After parsing:
  std::vector<Class> expandedClasses;
  bool hasSerialiable;
  TypeAttributesTable typeAttributes;

  /// constructor that parses interface file
  Module(const std::string& interfacePath, const std::string& moduleName,
      bool enable_verbose = true);

  /// Dummy constructor that does no parsing - use only for testing
  Module(const std::string& moduleName, bool enable_verbose = true);

  /// non-const function that performs parsing - typically called by constructor
  /// Throws exception on failure
  void parseMarkup(const std::string& data);

  /// MATLAB code generation:
  void matlab_code(const std::string& path) const;

  void generateIncludes(FileWriter& file) const;

  void finish_wrapper(FileWriter& file,
      const std::vector<std::string>& functionNames) const;

  /// Python code generation:
  void python_wrapper(const std::string& path) const;

private:
  static std::vector<Class> ExpandTypedefInstantiations(
      const std::vector<Class>& classes,
      const std::vector<TemplateInstantiationTypedef> instantiations);
  static std::vector<std::string> GenerateValidTypes(
      const std::vector<Class>& classes,
      const std::vector<ForwardDeclaration> forwardDeclarations);
  static void WriteCollectorsAndCleanupFcn(FileWriter& wrapperFile,
      const std::string& moduleName, const std::vector<Class>& classes);
  static void WriteRTTIRegistry(FileWriter& wrapperFile,
      const std::string& moduleName, const std::vector<Class>& classes);
};

} // \namespace wrap
