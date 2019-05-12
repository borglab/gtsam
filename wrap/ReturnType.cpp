/**
 * @file ReturnType.cpp
 * @date Nov 13, 2014
 * @author Frank Dellaert
 */

#include "ReturnType.h"
#include "Class.h"
#include "utilities.h"
#include <iostream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void ReturnType::wrap_result(const string& out, const string& result,
                             FileWriter& wrapperFile,
                             const TypeAttributesTable& typeAttributes) const {
  string cppType = qualifiedName("::"), matlabType = qualifiedName(".");

  if (category == CLASS) {
    // Handle Classes
    string objCopy, ptrType;
    const bool isVirtual = typeAttributes.attributes(cppType).isVirtual;
    if (isPtr)
      objCopy = result;  // a shared pointer can always be passed as is
    else {
      // but if we want an actual new object, things get more complex
      if (isVirtual)
        // A virtual class needs to be cloned, so the whole hierarchy is
        // returned
        objCopy = result + ".clone()";
      else {
        // ...but a non-virtual class can just be copied
        objCopy = "boost::make_shared<" + cppType + ">(" + result + ")";
      }
    }
    // e.g. out[1] = wrap_shared_ptr(pairResult.second,"gtsam.Point3", false);
    wrapperFile.oss << out << " = wrap_shared_ptr(" << objCopy << ",\""
                    << matlabType << "\", " << (isVirtual ? "true" : "false")
                    << ");\n";

  } else if (isPtr) {
    // Handle shared pointer case for BASIS/EIGEN/VOID
    // This case does not actually occur in GTSAM wrappers, so untested!
    wrapperFile.oss << "  {\n  boost::shared_ptr<" << qualifiedName("::")
                    << "> shared(" << result << ");" << endl;
    wrapperFile.oss << out << " = wrap_shared_ptr(shared,\"" << matlabType
                    << "\");\n  }\n";

  } else if (matlabType != "void")
    // Handle normal case case for BASIS/EIGEN
    wrapperFile.oss << out << " = wrap< " << qualifiedName("::") << " >(" << result
                    << ");\n";
}

/* ************************************************************************* */
void ReturnType::emit_cython_pxd(
    FileWriter& file, const std::string& className,
    const std::vector<std::string>& templateArgs) const {
  string cythonType;
  if (name() == "This")
    cythonType = className;
  else if (match(templateArgs))
    cythonType = name();
  else
    cythonType = pxdClassName();
  if (isPtr) cythonType = "shared_ptr[" + cythonType + "]";
  file.oss << cythonType;
}

/* ************************************************************************* */
std::string ReturnType::pyx_returnType(bool addShared) const {
  string retType = pxd_class_in_pyx();
  if (isPtr || (isNonBasicType() && addShared))
    retType = "shared_ptr[" + retType + "]";
  return retType;
}

/* ************************************************************************* */
std::string ReturnType::pyx_casting(const std::string& var,
                                    bool isSharedVar) const {
  if (isEigen()) {
    string s = "ndarray_copy(" + var + ")";
    if (pyxClassName() == "Vector")
      return s + ".squeeze()";
    else return s;
  }
  else if (isNonBasicType()) {
    if (isPtr || isSharedVar)
      return pyxClassName() + ".cyCreateFromShared(" + var + ")";
    else {
      // construct a shared_ptr if var is not a shared ptr
      return pyxClassName() + ".cyCreateFromShared(" + make_shared_pxd_class_in_pyx() +
             + "(" + var + "))";
    }
  } else
    return var;
}

/* ************************************************************************* */
