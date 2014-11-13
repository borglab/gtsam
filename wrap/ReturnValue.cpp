/**
 * @file ReturnValue.cpp
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Andrew Melim
 * @author Richard Roberts
 */

#include "ReturnValue.h"
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
    FileWriter& file, const TypeAttributesTable& typeAttributes) const {

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
    file.oss << out << " = wrap_shared_ptr(" << objCopy << ",\"" << matlabType
        << "\", " << (isVirtual ? "true" : "false") << ");\n";
  } else if (isPtr) {
    file.oss << "  Shared" << name << "* ret = new Shared" << name << "("
        << result << ");" << endl;
    file.oss << out << " = wrap_shared_ptr(ret,\"" << matlabType << "\");\n";
  } else if (matlabType != "void")
    file.oss << out << " = wrap< " << str(false) << " >(" << result << ");\n";
}

/* ************************************************************************* */
void ReturnType::wrapTypeUnwrap(FileWriter& wrapperFile) const {
  if (category == CLASS)
    wrapperFile.oss << "  typedef boost::shared_ptr<" << qualifiedName("::")
        << "> Shared" << name << ";" << endl;
}

/* ************************************************************************* */
ReturnValue ReturnValue::expandTemplate(const string& templateArg,
    const Qualified& qualifiedType, const Qualified& expandedClass) const {
  ReturnValue instRetVal = *this;
  if (type1.name == templateArg) {
    instRetVal.type1.rename(qualifiedType);
  } else if (type1.name == "This") {
    instRetVal.type1.rename(expandedClass);
  }
  if (type2.name == templateArg) {
    instRetVal.type2.rename(qualifiedType);
  } else if (type2.name == "This") {
    instRetVal.type2.rename(expandedClass);
  }
  return instRetVal;
}

/* ************************************************************************* */
string ReturnValue::return_type(bool add_ptr) const {
  if (isPair)
    return "pair< " + type1.str(add_ptr) + ", " + type2.str(add_ptr) + " >";
  else
    return type1.str(add_ptr);
}

/* ************************************************************************* */
string ReturnValue::matlab_returnType() const {
  return isPair ? "[first,second]" : "result";
}

/* ************************************************************************* */
void ReturnValue::wrap_result(const string& result, FileWriter& wrapperFile,
    const TypeAttributesTable& typeAttributes) const {
  if (isPair) {
    // For a pair, store the returned pair so we do not evaluate the function twice
    wrapperFile.oss << "  " << return_type(true) << " pairResult = " << result
        << ";\n";
    type1.wrap_result("  out[0]", "pairResult.first", wrapperFile,
        typeAttributes);
    type2.wrap_result("  out[1]", "pairResult.second", wrapperFile,
        typeAttributes);
  } else { // Not a pair
    type1.wrap_result("  out[0]", result, wrapperFile, typeAttributes);
  }
}

/* ************************************************************************* */
void ReturnValue::wrapTypeUnwrap(FileWriter& wrapperFile) const {
  type1.wrapTypeUnwrap(wrapperFile);
  if (isPair)
    type2.wrapTypeUnwrap(wrapperFile);
}

/* ************************************************************************* */
void ReturnValue::emit_matlab(FileWriter& proxyFile) const {
  string output;
  if (isPair)
    proxyFile.oss << "[ varargout{1} varargout{2} ] = ";
  else if (type1.category != ReturnType::VOID)
    proxyFile.oss << "varargout{1} = ";
}

/* ************************************************************************* */

