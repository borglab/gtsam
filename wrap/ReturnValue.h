/**
 * @file ReturnValue.h
 *
 * @brief Encapsulates a return value from a method
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Richard Roberts
 */

#include "ReturnType.h"
#include "TemplateSubstitution.h"
#include "FileWriter.h"
#include "TypeAttributesTable.h"
#include "utilities.h"

#pragma once

namespace wrap {

/**
 * Encapsulates return type of a method or function, possibly a pair
 */
struct ReturnValue {

  bool isPair;
  ReturnType type1, type2;

  friend struct ReturnValueGrammar;

  /// Constructor
  ReturnValue() :
      isPair(false) {
  }

  /// Constructor
  ReturnValue(const ReturnType& type) :
      isPair(false), type1(type) {
  }

  /// Constructor
  ReturnValue(const ReturnType& t1, const ReturnType& t2) :
      isPair(true), type1(t1), type2(t2) {
  }

  virtual void clear() {
    type1.clear();
    type2.clear();
    isPair = false;
  }

  bool operator==(const ReturnValue& other) const {
    return isPair == other.isPair && type1 == other.type1
        && type2 == other.type2;
  }

  /// Substitute template argument
  ReturnValue expandTemplate(const TemplateSubstitution& ts) const;

  std::string return_type(bool add_ptr) const;

  std::string matlab_returnType() const;

  void wrap_result(const std::string& result, FileWriter& wrapperFile,
      const TypeAttributesTable& typeAttributes) const;

  void wrapTypeUnwrap(FileWriter& wrapperFile) const;

  void emit_matlab(FileWriter& proxyFile) const;

  friend std::ostream& operator<<(std::ostream& os, const ReturnValue& r) {
    if (!r.isPair && r.type1.category == ReturnType::VOID)
      os << "void";
    else
      os << r.return_type(true);
    return os;
  }

};

} // \namespace wrap
