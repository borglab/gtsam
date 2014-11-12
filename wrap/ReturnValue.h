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
struct ReturnType: Qualified {

  /// the different supported return value categories
  typedef enum {
    CLASS = 1, EIGEN = 2, BASIS = 3, VOID = 4
  } return_category;

  bool isPtr;
  return_category category;

  ReturnType() :
      isPtr(false), category(CLASS) {
  }

  void rename(const Qualified& q) {
    name = q.name;
    namespaces = q.namespaces;
  }

  /// Check if this type is in a set of valid types
  template<class TYPES>
  void verify(TYPES validtypes, const std::string& s) const {
    std::string key = qualifiedName("::");
    if (find(validtypes.begin(), validtypes.end(), key) == validtypes.end())
      throw DependencyMissing(key, s);
  }

private:

  friend struct ReturnValue;

  std::string str(bool add_ptr) const;

  /// Example: out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
  void wrap_result(const std::string& out, const std::string& result,
      FileWriter& file, const TypeAttributesTable& typeAttributes) const;

  /// Creates typedef
  void wrapTypeUnwrap(FileWriter& wrapperFile) const;

};

/**
 * Encapsulates return value of a method or function, possibly a pair
 */
struct ReturnValue {

  bool isPair;
  ReturnType type1, type2;

  /// Constructor
  ReturnValue() :
      isPair(false) {
  }

  std::string return_type(bool add_ptr) const;

  std::string matlab_returnType() const;

  void wrap_result(const std::string& result, FileWriter& wrapperFile,
      const TypeAttributesTable& typeAttributes) const;

  void wrapTypeUnwrap(FileWriter& wrapperFile) const;

  void emit_matlab(FileWriter& proxyFile) const;
};

} // \namespace wrap
