/**
 * @file ReturnValue.cpp
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Andrew Melim
 * @author Richard Roberts
 */

#include "ReturnValue.h"
#include "utilities.h"
#include <iostream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
ReturnValue ReturnValue::expandTemplate(const TemplateSubstitution& ts) const {
  ReturnValue instRetVal = *this;
  instRetVal.type1 = ts.tryToSubstitite(type1);
  if (isPair)
    instRetVal.type2 = ts.tryToSubstitite(type2);
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

