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

#include <vector>

#pragma once

namespace wrap {

/**
 * Encapsulates return value of a method or function
 */
struct ReturnValue {

  /// the different supported return value categories
  typedef enum {
    CLASS = 1, EIGEN = 2, BASIS = 3, VOID = 4
  } return_category;

  bool isPtr1, isPtr2, isPair;
  return_category category1, category2;
  std::string type1, type2;
  std::vector<std::string> namespaces1, namespaces2;

  /// Constructor
  ReturnValue() :
      isPtr1(false), isPtr2(false), isPair(false), category1(CLASS), category2(
          CLASS) {
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
