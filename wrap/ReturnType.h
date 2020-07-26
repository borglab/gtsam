/**
 * @file ReturnValue.h
 * @brief Encapsulates a return type of a method
 * @date Nov 13, 2014
 * @author Frank Dellaert
 */

#include "Qualified.h"
#include "FileWriter.h"
#include "TypeAttributesTable.h"
#include "utilities.h"
#include <iostream>

#pragma once

namespace wrap {

/**
 * Encapsulates return value of a method or function
 */
struct ReturnType : public Qualified {
  bool isPtr;

  friend struct ReturnValueGrammar;

  /// Makes a void type
  ReturnType() : isPtr(false) {}

  /// Constructor, no namespaces
  ReturnType(const std::string& name, Category c = CLASS, bool ptr = false)
      : Qualified(name, c), isPtr(ptr) {}

  void clear() override {
    Qualified::clear();
    isPtr = false;
  }

  /// Check if this type is in a set of valid types
  template <class TYPES>
  void verify(TYPES validtypes, const std::string& s) const {
    std::string key = qualifiedName("::");
    if (find(validtypes.begin(), validtypes.end(), key) == validtypes.end())
      throw DependencyMissing(key, "checking return type of " + s);
  }

  /// @param className the actual class name to use when "This" is specified
  void emit_cython_pxd(FileWriter& file, const std::string& className,
                       const std::vector<std::string>& templateArgs) const;

  std::string pyx_returnType(bool addShared = true) const;
  std::string pyx_casting(const std::string& var,
                          bool isSharedVar = true) const;

private:
  friend struct ReturnValue;

  /// Example: out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
  void wrap_result(const std::string& out, const std::string& result,
                   FileWriter& wrapperFile,
                   const TypeAttributesTable& typeAttributes) const;
};

//******************************************************************************
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct ReturnTypeGrammar : public classic::grammar<ReturnTypeGrammar> {
  wrap::ReturnType& result_;  ///< successful parse will be placed in here

  TypeGrammar type_g;

  /// Construct ReturnType grammar and specify where result is placed
  ReturnTypeGrammar(wrap::ReturnType& result)
      : result_(result), type_g(result_) {}

  /// Definition of type grammar
  template <typename ScannerT>
  struct definition {
    classic::rule<ScannerT> type_p;

    definition(ReturnTypeGrammar const& self) {
      using namespace classic;
      type_p = self.type_g >> !ch_p('*')[assign_a(self.result_.isPtr, T)];
    }

    classic::rule<ScannerT> const& start() const { return type_p; }
  };
};
// ReturnTypeGrammar

}  // \namespace wrap
