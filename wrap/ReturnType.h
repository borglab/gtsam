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

  ReturnType(const std::string& name) :
      isPtr(false), category(CLASS) {
    Qualified::name = name;
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
      throw DependencyMissing(key, "checking return type of " + s);
  }

private:

  friend struct ReturnValue;

  std::string str(bool add_ptr) const;

  /// Example: out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
  void wrap_result(const std::string& out, const std::string& result,
      FileWriter& wrapperFile, const TypeAttributesTable& typeAttributes) const;

  /// Creates typedef
  void wrapTypeUnwrap(FileWriter& wrapperFile) const;

};

} // \namespace wrap
