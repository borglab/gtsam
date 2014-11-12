/**
 * @file ReturnValue.h
 *
 * @brief Encapsulates a return value from a method
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Richard Roberts
 */

#include "FileWriter.h"
#include "TypeAttributesTable.h"
#include "Qualified.h"

#pragma once

namespace wrap {

/**
 * Encapsulates return value of a method or function
 */
struct ReturnType : Qualified {

  ReturnType(): isPtr(false), category(CLASS) {
  }

  ReturnType(const Qualified& q): Qualified(q), isPtr(false), category(CLASS) {
  }

  /// the different supported return value categories
  typedef enum {
    CLASS = 1, EIGEN = 2, BASIS = 3, VOID = 4
  } return_category;

  bool isPtr;
  return_category category;
};

/**
 * Encapsulates return value of a method or function, possibly a pair
 */
struct ReturnValue {

  bool isPair;
  ReturnType type1, type2;

  /// Constructor
  ReturnValue() : isPair(false)  {
  }

  typedef enum {
    arg1, arg2, pair
  } pairing;

  std::string return_type(bool add_ptr, pairing p) const;

  std::string qualifiedType1(const std::string& delim = "") const;
  std::string qualifiedType2(const std::string& delim = "") const;

  std::string matlab_returnType() const;

  void wrap_result(const std::string& result, FileWriter& file,
      const TypeAttributesTable& typeAttributes) const;

  void wrapTypeUnwrap(FileWriter& wrapperFile) const;

  void emit_matlab(FileWriter& file) const;
};

} // \namespace wrap
