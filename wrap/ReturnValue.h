/**
 * @file ReturnValue.h
 *
 * @brief Encapsulates a return value from a method
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Richard Roberts
 */

#include "Qualified.h"
#include "FileWriter.h"
#include "TypeAttributesTable.h"
#include "utilities.h"

#pragma once

namespace wrap {

/**
 * Encapsulates return value of a method or function
 */
struct ReturnType : Qualified {

  /// the different supported return value categories
  typedef enum {
    CLASS = 1, EIGEN = 2, BASIS = 3, VOID = 4
  } return_category;

  bool isPtr;
  return_category category;

  ReturnType(): isPtr(false), category(CLASS) {
  }

  ReturnType(const Qualified& q): Qualified(q), isPtr(false), category(CLASS) {
  }

  std::string return_type(bool add_ptr) const;

  void wrap_result(const std::string& result, FileWriter& file,
      const TypeAttributesTable& typeAttributes) const;

  void wrapTypeUnwrap(FileWriter& wrapperFile) const;

  /// Check if this type is in a set of valid types
  template <class TYPES>
  void verify(TYPES validtypes, const std::string& s) const {
    std::string key = qualifiedName("::");
    if (find(validtypes.begin(), validtypes.end(), key) == validtypes.end())
      throw DependencyMissing(key, s);
  }

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

  std::string return_type(bool add_ptr) const;

  std::string matlab_returnType() const;

  void wrap_result(const std::string& result, FileWriter& file,
      const TypeAttributesTable& typeAttributes) const;

  void wrapTypeUnwrap(FileWriter& wrapperFile) const;

  void emit_matlab(FileWriter& file) const;
};

} // \namespace wrap
