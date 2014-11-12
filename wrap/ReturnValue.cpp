/**
 * @file ReturnValue.cpp
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Andrew Melim
 * @author Richard Roberts
 */

#include <boost/foreach.hpp>

#include "ReturnValue.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
string ReturnType::return_type(bool add_ptr) const {
  return maybe_shared_ptr(add_ptr || isPtr, qualifiedName("::"), name);
}

/* ************************************************************************* */
void ReturnType::wrap_result(const string& result, FileWriter& file,
    const TypeAttributesTable& typeAttributes) const {

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
    file.oss << "  out[0] = wrap_shared_ptr(" << objCopy << ",\"" << matlabType
        << "\", " << (isVirtual ? "true" : "false") << ");\n";
  } else if (isPtr) {
    file.oss << "  Shared" << name << "* ret = new Shared" << name << "("
        << result << ");" << endl;
    file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType << "\");\n";
  } else if (matlabType != "void")
    file.oss << "  out[0] = wrap< " << return_type(true) << " >(" << result
        << ");\n";
}

/* ************************************************************************* */
void ReturnType::wrapTypeUnwrap(FileWriter& file) const {
  if (category == CLASS)
    file.oss << "  typedef boost::shared_ptr<" << qualifiedName("::")
        << "> Shared" << name << ";" << endl;
}

/* ************************************************************************* */
string ReturnValue::return_type(bool add_ptr) const {
  return "pair< " + type1.return_type(add_ptr) + ", "
      + type2.return_type(add_ptr) + " >";
}

/* ************************************************************************* */
string ReturnValue::matlab_returnType() const {
  return isPair ? "[first,second]" : "result";
}

/* ************************************************************************* */
void ReturnValue::wrap_result(const string& result, FileWriter& file,
    const TypeAttributesTable& typeAttributes) const {
  if (isPair) {
    // For a pair, store the returned pair so we do not evaluate the function twice
    file.oss << "  " << return_type(false) << " pairResult = " << result
        << ";\n";
    type1.wrap_result("pairResult.first", file, typeAttributes);
    type2.wrap_result("pairResult.second", file, typeAttributes);
  } else { // Not a pair
    type1.wrap_result(result, file, typeAttributes);
  }
}

/* ************************************************************************* */
void ReturnValue::wrapTypeUnwrap(FileWriter& file) const {
  type1.wrapTypeUnwrap(file);
  if (isPair)
    type2.wrapTypeUnwrap(file);
}

/* ************************************************************************* */
void ReturnValue::emit_matlab(FileWriter& file) const {
  string output;
  if (isPair)
    file.oss << "[ varargout{1} varargout{2} ] = ";
  else if (type1.category != ReturnType::VOID)
    file.oss << "varargout{1} = ";
}

/* ************************************************************************* */

