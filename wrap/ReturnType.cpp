/**
 * @file ReturnType.cpp
 * @date Nov 13, 2014
 * @author Frank Dellaert
 */

#include "ReturnType.h"
#include "utilities.h"
#include <boost/foreach.hpp>
#include <iostream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
string ReturnType::str(bool add_ptr) const {
  return maybe_shared_ptr(add_ptr && isPtr, qualifiedName("::"), name);
}

/* ************************************************************************* */
void ReturnType::wrap_result(const string& out, const string& result,
    FileWriter& wrapperFile, const TypeAttributesTable& typeAttributes) const {

  string cppType = qualifiedName("::"), matlabType = qualifiedName(".");

  if (category == CLASS) {
    string objCopy, ptrType;
    ptrType = "Shared" + name;
    const bool isVirtual = typeAttributes.at(cppType).isVirtual;
    if (isVirtual) {
      if (isPtr)
        objCopy = result;
      else
        objCopy = result + ".clone()";
    } else {
      if (isPtr)
        objCopy = result;
      else
        objCopy = ptrType + "(new " + cppType + "(" + result + "))";
    }
    wrapperFile.oss << out << " = wrap_shared_ptr(" << objCopy << ",\""
        << matlabType << "\", " << (isVirtual ? "true" : "false") << ");\n";
  } else if (isPtr) {
    wrapperFile.oss << "  Shared" << name << "* ret = new Shared" << name << "("
        << result << ");" << endl;
    wrapperFile.oss << out << " = wrap_shared_ptr(ret,\"" << matlabType
        << "\");\n";
  } else if (matlabType != "void")
    wrapperFile.oss << out << " = wrap< " << str(false) << " >(" << result
        << ");\n";
}

/* ************************************************************************* */
void ReturnType::wrapTypeUnwrap(FileWriter& wrapperFile) const {
  if (category == CLASS)
    wrapperFile.oss << "  typedef boost::shared_ptr<" << qualifiedName("::")
        << "> Shared" << name << ";" << endl;
}

/* ************************************************************************* */

