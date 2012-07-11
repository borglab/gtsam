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
 **/

#pragma once

#include <string>
#include <vector>
#include <map>

#include "Class.h"

namespace wrap {

/**
 * A module just has a name and a list of classes
 */
struct Module {

	struct ForwardDeclaration {
		std::string name;
		bool isVirtual;
		ForwardDeclaration() : isVirtual(false) {}
	};

  std::string name;         ///< module name
  std::vector<Class> classes; ///< list of classes
  bool verbose;            ///< verbose flag
//  std::vector<std::string> using_namespaces; ///< all default namespaces
  std::vector<ForwardDeclaration> forward_declarations;

  /// constructor that parses interface file
  Module(const std::string& interfacePath,
	 const std::string& moduleName,
	 bool enable_verbose=true);

  /// MATLAB code generation:
  void matlab_code(
  		 const std::string& path,
		   const std::string& headerPath) const;

	void finish_wrapper(FileWriter& file, const std::vector<std::string>& functionNames) const;

	void generateIncludes(FileWriter& file) const;
};

} // \namespace wrap
