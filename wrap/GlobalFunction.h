/**
 * @file GlobalFunction.h
 *
 * @brief Implements codegen for a global function wrapped in matlab
 * 
 * @date Jul 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include "Function.h"

namespace wrap {

struct GlobalFunction: public Function {

  std::vector<Qualified> overloads; ///< Stack of qualified names

  // Constructor only used in Module
  GlobalFunction(bool verbose = true) :
      Function(verbose) {
  }

  // Used to reconstruct
  GlobalFunction(const std::string& name, bool verbose = true) :
      Function(name,verbose) {
  }

  // adds an overloaded version of this function,
  void addOverload(bool verbose, const Qualified& overload,
      const ArgumentList& args, const ReturnValue& retVal,
      const Qualified& instName = Qualified());

  // codegen function called from Module to build the cpp and matlab versions of the function
  void matlab_proxy(const std::string& toolboxPath,
      const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
      FileWriter& file, std::vector<std::string>& functionNames) const;

private:

  // Creates a single global function - all in same namespace
  void generateSingleFunction(const std::string& toolboxPath,
      const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
      FileWriter& file, std::vector<std::string>& functionNames) const;

};

} // \namespace wrap

