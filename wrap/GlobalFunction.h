/**
 * @file GlobalFunction.h
 *
 * @brief Implements codegen for a global function wrapped in matlab
 * 
 * @date Jul 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include "FullyOverloadedFunction.h"

namespace wrap {

struct GlobalFunction: public FullyOverloadedFunction {

  std::vector<Qualified> overloads; ///< Stack of qualified names

  // adds an overloaded version of this function,
  void addOverload(const Qualified& overload, const ArgumentList& args,
      const ReturnValue& retVal, const Qualified& instName = Qualified(),
      bool verbose = false);

  void verifyArguments(const std::vector<std::string>& validArgs) const {
    SignatureOverloads::verifyArguments(validArgs, name_);
  }

  void verifyReturnTypes(const std::vector<std::string>& validtypes) const {
    SignatureOverloads::verifyReturnTypes(validtypes, name_);
  }

  // codegen function called from Module to build the cpp and matlab versions of the function
  void matlab_proxy(const std::string& toolboxPath,
      const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
      FileWriter& file, std::vector<std::string>& functionNames) const;

  // emit python wrapper
  void python_wrapper(FileWriter& wrapperFile) const;

private:

  // Creates a single global function - all in same namespace
  void generateSingleFunction(const std::string& toolboxPath,
      const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
      FileWriter& file, std::vector<std::string>& functionNames) const;

};

} // \namespace wrap

